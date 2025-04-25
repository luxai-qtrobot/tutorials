
try:
    from llama_index.readers.web import SimpleWebPageReader
except ImportError:
    raise ImportError("The SimpleWebPageReader is not available. Please install the required package: `pip install llama-index-readers-web`")

from typing import List
from llama_index.core.readers.base import BaseReader
from data_loader.data_loader_base import DataLoaderBase
from utils.logger import Logger


class SimpleWebReader(DataLoaderBase):
    def __init__(self,                 
                 page_urls: List[str]
                 ):
        self.page_urls = page_urls

    def get_documents(self) -> BaseReader:
        Logger.info(f"Loading page from {self.page_urls} ...")
        documents =  SimpleWebPageReader(html_to_text=True).load_data(self.page_urls)
        for doc in documents:            
            Logger.info(f"loaded {doc.get_doc_id()}.")
            # Logger.debug(doc.get_content())
        return documents
    
