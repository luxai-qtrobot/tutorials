
from llama_index.core import SimpleDirectoryReader
from llama_index.core.readers.base import BaseReader
from data_loader.data_loader_base import DataLoaderBase
from utils.logger import Logger


class LocalFilesReader(DataLoaderBase):
    def __init__(self,                 
                 document_path="./",
                 document_formats=[".pdf"],
                 max_num_documents=5,
                 ):
        self.document_path = document_path
        self.document_formats = document_formats  
        self.max_num_documents = max_num_documents

    def get_documents(self) -> BaseReader:
        Logger.info(f"Loading {self.document_formats} documents from {self.document_path} ...")
        documents = SimpleDirectoryReader(input_dir=self.document_path,
                                     required_exts=self.document_formats,
                                     num_files_limit=self.max_num_documents).load_data(num_workers=1)
        for doc in documents:
            meta = doc.metadata              
            Logger.info(f"loaded {meta['file_type']} {meta['file_path']} page {meta.get('page_label', 'unknown')}")
        return documents
    