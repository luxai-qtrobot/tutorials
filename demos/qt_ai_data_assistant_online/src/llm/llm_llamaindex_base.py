# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from abc import abstractmethod
from typing import List, Tuple
from threading import Lock

from llama_index.core import Settings
from llama_index.core.memory import ChatMemoryBuffer
from llama_index.core.storage.chat_store import SimpleChatStore
from llama_index.core.llms import ChatMessage
from llama_index.core.chat_engine import SimpleChatEngine, ContextChatEngine
from llama_index.core.schema import MetadataMode, NodeWithScore, QueryBundle

from tenacity import retry, wait_random_exponential, stop_after_attempt

from data_loader.data_loader_base import DataLoaderBase
from utils.logger import Logger
from utils.utils import split_into_sentences
from vector_store.vector_store_base import VectorStoreBaseEngine


class CustomChatEngine(ContextChatEngine):
    """
    Custom Chat Engine inherited from ContextChatEngine
    """
    CUSTOM_CONTEXT_TEMPLATE = (
        "Context information is below."
        "\n--------------------\n"
        "{context_str}"        
        "\n--------------------\n\n"
        "Camera information is below."
        "\n--------------------\n"        
        "{camera_context_str}"
        "\n--------------------\n"
    )

    def __init__(self, *args, **kwargs):        
        super().__init__(*args, **kwargs)
        self.camera_context_str = ""


    def update_camera_context(self, scene_desc: str):
        self.camera_context_str = scene_desc


    def _generate_context(self, message: str) -> Tuple[str, List[NodeWithScore]]:
        """Generate context information from a message."""
        nodes = self._retriever.retrieve(message)
        for postprocessor in self._node_postprocessors:
            nodes = postprocessor.postprocess_nodes(
                nodes, query_bundle=QueryBundle(message)
            )

        context_str = "\n\n".join(
            [n.node.get_content(metadata_mode=MetadataMode.LLM).strip() for n in nodes]
        )         
        return self._context_template.format(context_str=context_str, camera_context_str=self.camera_context_str), nodes
    


class LLMLamaIndexBase:
    def __init__(self,                 
                 system_role="",
                 max_tokens=None, 
                 data_loader: DataLoaderBase = None,                 
                 reload_documents=False,
                 scene_procesing=False,
                 disable_rag=False,
                 mem_store_file=None):
        self.disable_rag = disable_rag
        self.system_role = system_role
        self.max_tokens = max_tokens
        self.scene_procesing = scene_procesing
        self.mem_store_file = mem_store_file
        self.data_loader = data_loader

        # setup llm engine
        Settings.llm = self.get_chat_engine()
        self.llm = Settings.llm
        Logger.info(f"LLMLamaIndexBase: using {self.llm.metadata.model_name} with {self.llm.metadata.context_window} context window.")

        self.chat_store = None
        if mem_store_file:
            Logger.info(f"LLMLamaIndexBase: loading memmory from {self.mem_store_file}...")
            self.chat_store = SimpleChatStore.from_persist_path(mem_store_file)
        
        self.memory = ChatMemoryBuffer.from_defaults(
            token_limit=max_tokens,
            chat_store=self.chat_store)
        self.chat_engine_lock = Lock()

        if scene_procesing:
            self.system_role = (
                f"{self.system_role}\n"
                f"You have access to a camera that provides scene descriptions.\n" 
                f"Only refer to the scene if the user explicitly asks about it.\n"
                f"Answer scene-related questions briefly and only if the camera information is available.\n"
                )


        if not disable_rag:
            Settings.embed_model = self.get_embedding_engine()                
            vector_store_engine = self.get_vector_store_engine()

            if vector_store_engine.is_empty() or reload_documents:                 
                # clear existing index if any 
                vector_store_engine.clear()                
                self.documents = self.data_loader.get_documents()
                self.index = vector_store_engine.get_index(self.documents)                
            else:
                Logger.info("Laoding existing index from storage...")
                self.index = vector_store_engine.get_index()


            self.retriever = self.index.as_retriever()
            self.chat_engine = CustomChatEngine.from_defaults(
                retriever=self.retriever,
                llm=self.llm,
                memory=self.memory,
                system_prompt=(self.system_role),
                context_template=CustomChatEngine.CUSTOM_CONTEXT_TEMPLATE,
                verbose=True,
                # tools=self.get_tools() if self.get_tools() else None                
            )
        else:            
            self.chat_engine = SimpleChatEngine.from_defaults(
                memory=self.memory,
                system_prompt=(self.system_role),
                # tools=self.get_tools() if self.get_tools() else None
                )


    def close(self):
        Logger.info("LLMLamaIndexBase: closing...")
        if self.chat_store:  
            Logger.info(f"LLMLamaIndexBase: storing memmory to {self.mem_store_file}...")
            self.chat_store.persist(self.mem_store_file)

    # def update_camera_feed(self, scene_desc:str):
    #     if not self.scene_procesing or not scene_desc:
    #         return
    #     with self.chat_engine_lock:
    #         if not self.disable_rag:
    #             self.chat_engine.update_camera_context(scene_desc)
    #         else:
    #             update_sys_role = (
    #                 f"{self.system_role}\n"
    #                 f"Camera information is below\n: {scene_desc.replace('image', 'camera feed')}\n"
    #                 )
    #             self.chat_engine = SimpleChatEngine.from_defaults(
    #                 memory=self.memory,
    #                 system_prompt=(update_sys_role))

    @retry(wait=wait_random_exponential(multiplier=1, max=40), stop=stop_after_attempt(3))
    def get_response(self, text):        
        with self.chat_engine_lock:
            resp = self.chat_engine.chat(text)        
        return resp
    
    def _process_response(self, response):        
        # Add the model's response to the history
        if response.choices[0].message:
            self.add_message(response.choices[0].message)                
        response_message = response.choices[0].message.content
        return response_message, []



    @retry(wait=wait_random_exponential(multiplier=1, max=40), stop=stop_after_attempt(3))
    def get_stream_response(self, text, user_id=None):        
        with self.chat_engine_lock:
            buffer = ''
            all = '' 
            user_id = user_id if user_id is not None else 'unknown'
            query = text
            if user_id:
                query = f"{text} (user {user_id})"
            response = self.chat_engine.stream_chat(query)
            for token in response.response_gen: 
                if not token:
                    continue   
                
                all += token
                buffer += token    
                sentences = split_into_sentences(buffer)
                sentences = [l for l in sentences if l.strip()]
                # Logger.info(sentences)
                if len(sentences) > 1:
                    yield (sentences[0].replace('\n', '').strip())
                    buffer = sentences[1]
            # yield any remaining buffer content as a final sentence
            if buffer.strip():            
                yield buffer.replace('\n', '').strip()
        

    def clear_memmory(self):
        with self.chat_engine_lock:
            self.chat_engine.reset()
        


    @retry(wait=wait_random_exponential(multiplier=1, max=40), stop=stop_after_attempt(3))
    def get_raw_chat(self, system:str, user:str) -> str:
        with self.chat_engine_lock:
            messages = [
                ChatMessage(role="system", content=system),
                ChatMessage(role="user", content=user),
            ]     
            response = self.llm.chat(messages)
        return response.message.content
        


    @abstractmethod
    def get_chat_engine(self):
        raise(NotImplementedError("get_llm_engine() must be implemented in the subclass."))
    
    @abstractmethod
    def get_embedding_engine(self):
        raise(NotImplementedError("get_llm_engine() must be implemented in the subclass."))

    @abstractmethod
    def get_vector_store_engine(self) -> VectorStoreBaseEngine:
        raise(NotImplementedError("get_llm_engine() must be implemented in the subclass."))
    
    @abstractmethod
    def get_tools(self):
        raise(NotImplementedError("get_tools() must be implemented in the subclass."))
