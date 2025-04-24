# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import datetime
from typing import Optional

try:
    from llama_index.llms.openai import OpenAI
    from llama_index.embeddings.openai import OpenAIEmbedding
    from llama_index.core.tools import FunctionTool
except ImportError:    
    raise ImportError("LlamaIndex OpenAI module not found. Please install it using 'pip install llama-index-embeddings-openai'")


from utils.logger import Logger
from llm.llm_llamaindex_base import LLMLamaIndexBase
from vector_store.vector_store_base import VectorStoreBaseEngine



# --- Define your custom function tool ---
def get_datetime():
    """Get the current date and time"""
    print('get_datetime called')
    return datetime.datetime.now().isoformat()


class LLMLamaIndexOpenAI(LLMLamaIndexBase):
    def __init__(self,                 
                 api_key,
                 model='gpt-4o-mini',
                 embedding_model='text-embedding-3-small',
                 vector_store_engine:Optional[VectorStoreBaseEngine]=None,
                 **kwargs):
        self.model = model
        self.api_key = api_key  
        self.embedding_model = embedding_model      
        self.vector_store_engine = vector_store_engine

        # create tools
        get_datetime_tool = FunctionTool.from_defaults(fn=get_datetime)
        self.custom_tools = [get_datetime_tool]
        print(get_datetime_tool.metadata)
        super().__init__(**kwargs)

    
    def get_chat_engine(self):
        return OpenAI(
            model=self.model,
            api_key=self.api_key,
            tools=self.get_tools() if self.get_tools() else None,
        )
    
    def get_embedding_engine(self):
        return OpenAIEmbedding(model=self.embedding_model, api_key=self.api_key)    
    
    def get_vector_store_engine(self) -> VectorStoreBaseEngine:
        return self.vector_store_engine

    def get_tools(self):
        return self.custom_tools
    
