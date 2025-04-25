
try:
    import faiss
    from llama_index.vector_stores.faiss import FaissVectorStore
    from llama_index.core.vector_stores.types import BasePydanticVectorStore
except ImportError:
    raise ImportError("LlamaIndex FAISS module not found. Please install it using 'pip install llama-index-vector-stores-faiss faiss-cpu'")


from llama_index.core.indices.base import BaseIndex
from llama_index.core.readers.base import BaseReader
from llama_index.core import StorageContext, VectorStoreIndex, load_index_from_storage
from vector_store.vector_store_base import VectorStoreBaseEngine
from utils.logger import Logger


class VectorStoreFAISS(VectorStoreBaseEngine):
    def __init__(self, 
                 persist_dir: str = "faiss_store",
                 embedding_dimension: int = 1536):
        
        self.persist_dir = persist_dir  
        self.embedding_dimension = embedding_dimension      

        try:
            self.vector_store = FaissVectorStore.from_persist_dir(persist_dir)
            self.storage_context = StorageContext.from_defaults(vector_store=self.vector_store, persist_dir=self.persist_dir)
        except Exception as e:
            Logger.info(f"VectorStoreFAISS: could not load index from {self.persist_dir}. recreating index...")
            self._recreate()            

    def _recreate(self):
        index = faiss.IndexFlatL2(self.embedding_dimension)
        self.vector_store = FaissVectorStore(faiss_index=index) 
        self.storage_context = StorageContext.from_defaults(vector_store=self.vector_store)
        

    def get_index(self, document: BaseReader = None) -> BaseIndex:
        if document:
            index =  VectorStoreIndex.from_documents(
                document,
                storage_context=self.storage_context
                )            
            try:
                self.storage_context.persist(self.persist_dir)
            except Exception as e: 
                Logger.error(f"VectorStoreFAISS: Error persisting vector store: {e}")             
            return index
        
        return load_index_from_storage(storage_context=self.storage_context)


    def is_empty(self):        
        return self.vector_store.client.ntotal == 0


    def clear(self):        
        try:                    
            self._recreate()
            # self.vector_store.client.reset()
            # self.storage_context.persist(self.persist_dir)
        except Exception as e:
            Logger.error(f"VectorStoreFAISS: Error clearing vector store: {e}")

