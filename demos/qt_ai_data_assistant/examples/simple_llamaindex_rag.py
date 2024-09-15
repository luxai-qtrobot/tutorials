# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
import os, sys
from termcolor import colored 

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from llamaindex_interface import ChatWithRAG
from llm_prompts import ConversationPrompt, WakeupPrompt

 
if __name__ == '__main__':
    
    current_dir = os.path.dirname(os.path.abspath(__file__))   
    default_documents_dir = os.path.abspath(os.path.join(current_dir, '..', 'documents'))

    model = 'llama3.1'    
    chat = ChatWithRAG(model=model, 
                       system_role=ConversationPrompt['system_role'],
                       document_path=default_documents_dir,
                       document_formats=['.pdf'])
    try:
        while True:
            user = input("User: ").strip()
            if not user:
                continue

            if user.lower() == 'exit':
                break        
            
            if user.lower() == 'forget_chat':
                chat.clear_memmory()
                continue
            
            if user.lower() == 'print_history':                
                print(colored(f"{chat.memory.json()}\n", 'magenta'))
                continue

            t = time.time()
            for msg in  chat.get_stream_response(text=user):            
                lat = int((time.time() - t)*1000)
                print(colored(f"{lat}ms", 'yellow'), colored(f"{msg}", 'blue'))
    except Exception as e:
        print(e)        
    
    chat.close()
