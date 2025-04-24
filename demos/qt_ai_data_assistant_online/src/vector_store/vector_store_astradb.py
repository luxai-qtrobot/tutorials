
try:
    from llama_index.vector_stores.astra_db import AstraDBVectorStore
    from llama_index.core.vector_stores.types import BasePydanticVectorStore
except ImportError:
    raise ImportError("LlamaIndex AstraDB module not found. Please install it using 'llama-index-vector-stores-astra-db'")

from vector_store.vector_store_base import VectorStoreBaseEngine
from utils.logger import Logger


class VectorStoreAstraDB(VectorStoreBaseEngine):
    def __init__(self,                 
                 token: str,
                 endpoint: str,
                 collection_name: str = "my_collection",
                 embedding_dimension: int = 1536,
                 ):
        self.collection_name = collection_name
        self.vector_store = AstraDBVectorStore(
            token=token,
            api_endpoint=endpoint,
            collection_name=collection_name,
            embedding_dimension=embedding_dimension
        )
        
    def get_store(self) -> BasePydanticVectorStore:
        return self.vector_store
    
    
    def is_empty(self) -> bool:
        try:
            index_count = self.vector_store._database.get_collection(self.collection_name).count_documents(filter={}, upper_bound=10)
            return index_count == 0
        except Exception as e:
            Logger.error(f"VectorStoreAstraDB: Error checking if vector store is empty: {e}")
            return True
    
    def clear(self):
        try:
            self.vector_store._database.get_collection(self.collection_name).delete_many(filter={})            
        except Exception as e:
            Logger.error(f"VectorStoreAstraDB: Error clearing vector store: {e}")        
