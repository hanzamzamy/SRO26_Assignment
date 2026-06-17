import os
import json
import io
import ast
import ollama

class OllamaNavigator:
    def __init__(self, config_file="config.json", memory_file="memory.json", model_name="gemma4:e2b"):
        # You can change model_name to "llama3.2-vision" or "llava" if benchmarking others
        self.memory_file = memory_file
        self.model_name = model_name 
        self.last_target_object = None # Store the clean object name for verification
        
        # Load Prompts
        with open(config_file, 'r') as f:
            self.config = json.load(f)
            
        print(f"[SYSTEM] Initialized Local Ollama Agent using model: {self.model_name}")

    def _load_memory(self):
        if not os.path.exists(self.memory_file):
            return {"spatial_memory": []}
        try:
            with open(self.memory_file, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError:
            return {"spatial_memory": []}

    def _save_memory(self, kb):
        with open(self.memory_file, 'w') as f:
            json.dump(kb, f, indent=2)

    def _pil_to_bytes(self, pil_image):
        """Ollama requires raw image bytes."""
        byte_stream = io.BytesIO()
        # Convert to RGB just in case it's RGBA to avoid JPEG errors
        if pil_image.mode != 'RGB':
            pil_image = pil_image.convert('RGB')
        pil_image.save(byte_stream, format='JPEG')
        return byte_stream.getvalue()

    def analyze_exploration_snapshot(self, pil_image, grid_idx, yaw):
        """Phase 1: Looks at an image, extracts landmarks, updates JSON."""
        prompt = f"{self.config['system_context']} \nTask: {self.config['exploration_prompt']}\nReply with just one short sentence."
        
        try:
            response = ollama.chat(
                model=self.model_name,
                messages=[{
                    'role': 'user',
                    'content': prompt,
                    'images': [self._pil_to_bytes(pil_image)]
                }]
            )
            description = response['message']['content'].strip()
            
            kb = self._load_memory()
            kb["spatial_memory"].append({"grid_idx": list(grid_idx), "yaw": float(yaw), "view": description})
            self._save_memory(kb)
            
            return description
        except Exception as e:
            return f"Error connecting to Ollama: {e}"

    def remove_invalid_memory(self, grid_idx):
        """Phase 2: If target is no longer there, delete the memory entry."""
        kb = self._load_memory()
        kb["spatial_memory"] = [m for m in kb["spatial_memory"] if m["grid_idx"] != list(grid_idx)]
        self._save_memory(kb)

    def get_execution_target(self, user_command):
        """Phase 2: Reads user command + JSON memory to determine target coordinates."""
        kb = self._load_memory()
        
        prompt = f"""
        {self.config['system_context']}
        
        User Command: "{user_command}"
        Spatial Memory Database: {json.dumps(kb)}
        
        Task:
        1. Identify the user's intent. If they are just chatting and NOT asking to navigate/search, set status to NOT_A_COMMAND.
        2. Extract the exact core object the user wants (e.g. if command is "move closer to the laptop", target_object is "laptop").
        3. If the object IS in the memory database, return FOUND_IN_MEMORY and its coords/yaw.
        4. If the object IS NOT in the database, return SEARCH_REQUESTED.
        
        Output ONLY a valid JSON object matching this exact schema. 
        CRITICAL RULES:
        - Do NOT wrap the JSON in markdown blocks or backticks.
        - You MUST use DOUBLE QUOTES ("") for all property names and string values. Never use single quotes ('').
        
        {{
            "status": "FOUND_IN_MEMORY" or "SEARCH_REQUESTED" or "NOT_A_COMMAND",
            "target_object": "string (the physical object)",
            "coords": [x, y] or null,
            "yaw": float or null,
            "reasoning": "string explaining your choice"
        }}
        """
        
        try:
            # We enforce JSON format natively through Ollama
            response = ollama.chat(
                model=self.model_name,
                messages=[{
                    'role': 'user',
                    'content': prompt
                }],
                format='json'
            )
            
            raw_content = response['message']['content'].strip()
            
            # 1. Isolate the JSON block (ignores conversational babble before/after)
            start_idx = raw_content.find('{')
            end_idx = raw_content.rfind('}')
            
            if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
                raw_content = raw_content[start_idx:end_idx+1]
            
            try:
                # 2. Try standard strict JSON parsing
                result = json.loads(raw_content)
            except json.JSONDecodeError as e:
                # 3. Fallback: If the model stubbornly used single quotes, it's a Python dict format.
                # json.loads() fails on single quotes, but ast.literal_eval parses them perfectly.
                try:
                    python_str = raw_content.replace('null', 'None').replace('true', 'True').replace('false', 'False')
                    result = ast.literal_eval(python_str)
                except Exception:
                    # If both parsers fail, raise the original JSON error to be caught by the outer block
                    raise e
            
            # Store the clean object name
            if result.get("target_object"):
                self.last_target_object = result["target_object"]
            else:
                self.last_target_object = user_command
                
            return result
            
        except Exception as e:
            return {"status": "NOT_A_COMMAND", "coords": None, "reasoning": f"Local Inference Error: {e}"}

    def verify_target_presence(self, pil_image, target_name):
        """Phase 2: Validates if the target actually exists at the coordinates."""
        # Use the cleanly extracted object name instead of the raw user prompt
        actual_target = getattr(self, 'last_target_object', target_name)
        if not actual_target:
            actual_target = target_name
            
        prompt = self.config['verification_prompt'].replace("{target}", actual_target)
        
        try:
            response = ollama.chat(
                model=self.model_name,
                messages=[{
                    'role': 'user',
                    'content': prompt,
                    'images': [self._pil_to_bytes(pil_image)]
                }]
            )
            return "YES" in response['message']['content'].upper()
        except Exception:
            return False