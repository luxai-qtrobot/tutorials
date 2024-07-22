import re
import json

from ollama import chat
from tenacity import retry, wait_random_exponential, stop_after_attempt



class ChatWithMemory:
    def __init__(self,  model, system_role="", max_tokens=3e3, max_converstion_memmory=10):        
        self.model = model
        self.system_role = system_role
        self.history = [ {"role": "system", "content": system_role} ]
        self.total_used_token = 0
        self.max_tokens = max_tokens
        self.max_converstion_memmory = max_converstion_memmory


    def update_sys_prompt(self, description):
        if not description:
            return        
        sys_content = f"{self.system_role}\n{description}"                       
        self.history[0]['content'] = sys_content
       

    def add_message(self, message):
        if not message:
            return
        self.history.append(message)

        # limit the chat pair memory (user-assitant) to the max_converstion_memmory 
        if message.get('role') == 'assistant':
            self.history = self._limit_memory(self.history, self.max_converstion_memmory)
            # trim the history if required by removing old messages
            while self._num_tokens_from_messages() > self.max_tokens:
                if len(self.history) > 2:
                    self.history.pop(1)
                    self.history.pop(1)


    def _limit_memory(self, memory, limit):
        if len(memory) < 3:
            return memory
        # Extract the system message
        system_message = [message for message in memory if message['role'] == 'system']        
        # Extract user and assistant messages
        user_assistant_messages = [message for message in memory if message['role'] in ['user', 'assistant']]    
        # Pair the user and assistant messages
        paired_messages = [(user_assistant_messages[i], user_assistant_messages[i+1]) for i in range(0, len(user_assistant_messages), 2)]        
        # Keep only the last 'limit' pairs
        limited_pairs = paired_messages[-limit:]        
        # Flatten the list of pairs back into individual messages
        limited_messages = [message for pair in limited_pairs for message in pair]        
        # Combine the system message with the limited user/assistant messages
        limited_memory = system_message + limited_messages        
        return limited_memory


    @retry(wait=wait_random_exponential(multiplier=1, max=40), stop=stop_after_attempt(3))
    def get_response(self, text):
        self.add_message({"role": "user", "content": text})        
        response = chat(model=self.model, messages=self.history)
        # process response
        return self._process_response(response)
    
    def _process_response(self, response):        
        # Add the model's response to the history
        if response.choices[0].message:
            self.add_message(response.choices[0].message)                
        response_message = response.choices[0].message.content
        return response_message, []



    @retry(wait=wait_random_exponential(multiplier=1, max=40), stop=stop_after_attempt(3))
    def get_stream_response(self, text):
        self.add_message({"role": "user", "content": text})        
        response = chat(model=self.model, 
                        messages=self.history, 
                        stream=True)
        # sentence_end_pattern = re.compile(r'(?<!\w\.\w.)(?<![A-Z][a-z]\.)(?<=\.|\?|!|\n)')                
        sentence_end_pattern = re.compile(r'(?<!\w\.\w.)(?<![A-Z][a-z]\.)(?<=\.|\?|!|\n)\s*')        
        buffer = ''
        response_message = ''
        for chunk in response:            
            token = chunk['message']['content']
            if not token:
                continue            
            buffer += token
            response_message += token
            while True:
                match = sentence_end_pattern.search(buffer)                
                if match:                    
                    end = match.end()
                    sentence = buffer[:end].strip()                    
                    yield sentence                    
                    buffer = buffer[end:].strip()                    
                else:
                    break

        # yield any remaining buffer content as a final sentence
        if buffer.strip():
            yield buffer

        # Add the model's response to the history         
        if response_message.strip():
            self.add_message({"role": "assistant", "content": response_message.strip()})



    # TODO: this must be updated to precisely count tokens
    def _num_tokens_from_messages(self):
        """Return the number of tokens used by a list of messages."""        
        tokens_per_message = 3  
        tokens_per_name = 1 
        num_tokens = 0
        for message in self.history:
            num_tokens += tokens_per_message            
            if isinstance(message, dict):                
                for key, value in message.items():                    
                    value_string = json.dumps(value) if not isinstance(value, str) else value
                    num_tokens += len((value_string))
                    if key == "name":
                        num_tokens += tokens_per_name
            else: # isinstance(message, ChatCompletion): 
                num_tokens += len(str(message.content))
        num_tokens += 3  # every reply is primed with <|start|>assistant<|message|>
        self.total_used_token = num_tokens
        return num_tokens


    def clear_memmory(self):
        self.history = [ {"role": "system", "content": self.system_role} ]
        self.total_used_token = 0

