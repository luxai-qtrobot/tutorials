# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import json
from typing import Any, List, Optional, Tuple
from threading import Lock
from llama_index.core import VectorStoreIndex, SimpleDirectoryReader, Settings, Document
from llama_index.embeddings.ollama import OllamaEmbedding
from llama_index.llms.ollama import Ollama
from llama_index.core.memory import ChatMemoryBuffer
from llama_index.core.storage.chat_store import SimpleChatStore
from llama_index.core.llms import ChatMessage
from llama_index.core.chat_engine import SimpleChatEngine, ContextChatEngine
from llama_index.core.schema import MetadataMode, NodeWithScore, QueryBundle
# from llama_index.core import load_index_from_storage, StorageContext
# from llama_index.core.storage.docstore import SimpleDocumentStore
# from llama_index.core.vector_stores import SimpleVectorStore
# from llama_index.core.storage.index_store import SimpleIndexStore

from tenacity import retry, wait_random_exponential, stop_after_attempt

from termcolor import colored


from utils import split_into_sentences

def pretty_print(item):
    print(json.dumps(item, indent=2))


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
        # Call the parent class's constructor
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
    

class ChatWithRAG:
    def __init__(self,
                 model,
                 system_role="",
                 max_tokens=4096, 
                 document_path="./",
                 document_formats=[".pdf"],
                 max_num_documents=5,
                 scene_procesing=False,
                 disable_rag=False,
                 mem_store_file=None):
        self.model = model
        self.disable_rag = disable_rag
        self.system_role = system_role
        self.max_tokens = max_tokens
        self.scene_procesing = scene_procesing
        self.mem_store_file = mem_store_file

        Settings.llm = Ollama(model=model, request_timeout=300.0)
        self.llm = Settings.llm

        self.chat_store = None
        if mem_store_file:
            print(f"ChatWithRAG: loading memmory from {self.mem_store_file}...")
            self.chat_store = SimpleChatStore.from_persist_path(mem_store_file)
        
        self.memory = ChatMemoryBuffer.from_defaults(
            token_limit=max_tokens,
            chat_store=self.chat_store)
        self.chat_engine_lock = Lock()

        if scene_procesing:
            # scene_doc = Document( metadata={'category': 'Here is what you see now', 'filename':'camera feed', 'type': 'description of visual input'}, text='')
            # self.documents.insert(0, scene_doc)                
            self.system_role = (
                f"{self.system_role}\n"
                f"You have access to a camera that provides scene descriptions.\n" 
                f"Only refer to the scene if the user explicitly asks about it.\n"
                f"Answer scene-related questions briefly and only if the camera information is available.\n"
                )


        if not disable_rag:
            print(f"loading documents from {document_path}...")
            self.documents = SimpleDirectoryReader(input_dir=document_path,
                                                required_exts=document_formats,
                                                num_files_limit=max_num_documents).load_data(num_workers=1)        
            for doc in self.documents:
                meta = doc.metadata              
                print(f"loaded {meta['file_type']} {meta['file_path']} page {meta.get('page_label', 'unknown')}")

            Settings.embed_model = OllamaEmbedding("mxbai-embed-large:latest")
            self.index = VectorStoreIndex.from_documents(self.documents)
            self.retriever = self.index.as_retriever()
            self.chat_engine = CustomChatEngine.from_defaults(
                retriever=self.retriever,
                llm=self.llm,
                memory=self.memory,
                system_prompt=(self.system_role),
                context_template=CustomChatEngine.CUSTOM_CONTEXT_TEMPLATE,
                verbose=True
            )
        else:            
            self.chat_engine = SimpleChatEngine.from_defaults(
                memory=self.memory,
                system_prompt=(self.system_role))


    def close(self):
        print("ChatWithRAG: closing...")
        if self.chat_store:  
            print(f"ChatWithRAG: storing memmory to {self.mem_store_file}...")
            self.chat_store.persist(self.mem_store_file)

    def update_camera_feed(self, scene_desc:str):
        if not self.scene_procesing or not scene_desc:
            return
        with self.chat_engine_lock:
            if not self.disable_rag:
                self.chat_engine.update_camera_context(scene_desc)
            else:
                update_sys_role = (
                    f"{self.system_role}\n"
                    f"Camera information is below\n: {scene_desc.replace('image', 'camera feed')}\n"
                    )
                self.chat_engine = SimpleChatEngine.from_defaults(
                    memory=self.memory,
                    system_prompt=(update_sys_role))

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
                # print(sentences)
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
        
