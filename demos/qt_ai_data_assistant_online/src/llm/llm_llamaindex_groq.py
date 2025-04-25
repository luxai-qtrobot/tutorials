# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

try:
    from llama_index.llms.groq import Groq
except ImportError:    
    raise ImportError("LlamaIndex Groq module not found. Please install it using 'pip install llama-index-llms-groq'")
try:
    from llama_index.embeddings.huggingface import HuggingFaceEmbedding
except ImportError:
    raise ImportError("LlamaIndex HuggingFace module not found. Please install it using 'pip install llama-index-embeddings-huggingface'")


from typing import Optional
from utils.logger import Logger
from llm.llm_llamaindex_base import LLMLamaIndexBase
from vector_store.vector_store_base import VectorStoreBaseEngine

class LLMLamaIndexGroq(LLMLamaIndexBase):
    def __init__(self,                 
                 api_key,
                 model='llama-3.1-8b-instant',
                 embedding_model='BAAI/bge-small-en-v1.5',
                 vector_store_engine:Optional[VectorStoreBaseEngine]=None,
                 **kwargs):
        self.model = model
        self.api_key = api_key  
        self.embedding_model = embedding_model      
        self.vector_store_engine = vector_store_engine
        super().__init__(**kwargs)

    
    def get_chat_engine(self):        
        return Groq(model=self.model, api_key=self.api_key)
        
    def get_embedding_engine(self):
         return HuggingFaceEmbedding(model_name=self.embedding_model)
    
    def get_vector_store_engine(self) -> VectorStoreBaseEngine:
        return self.vector_store_engine

    def get_tools(self):
        return []