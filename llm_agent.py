import os
import json
from google import genai
from google.genai import types

class GeminiNavigator:
    def __init__(self, config_file="config.json", memory_file="memory.json"):
        # Initialize the modern SDK client (Automatically picks up GEMINI_API_KEY from env)
        try:
            self.client = genai.Client()
        except Exception as e:
            print(f"[ERROR] Failed to init GenAI Client. Did you set GEMINI_API_KEY? Error: {e}")
            
        self.memory_file = memory_file
        self.model_name = 'gemma-4-31b-it' 
        self.last_target_object = None # Store the clean object name for verification
        
        # Load Prompts
        with open(config_file, 'r') as f:
            self.config = json.load(f)

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

    def analyze_exploration_snapshot(self, pil_image, grid_idx, yaw):
        """Phase 1: Looks at an image, extracts landmarks, updates JSON."""
        prompt = f"{self.config['system_context']} \nTask: {self.config['exploration_prompt']}"
    
        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[pil_image, prompt]
            )
            description = response.text.strip()
            
            kb = self._load_memory()
            kb["spatial_memory"].append({"grid_idx": grid_idx, "yaw": yaw, "view": description})
            self._save_memory(kb)
            
            return description
        except Exception as e:
            return f"Error connecting to Gemini: {e}"

    def remove_invalid_memory(self, grid_idx):
        """Phase 2: If target is no longer there, delete the memory entry."""
        kb = self._load_memory()
        kb["spatial_memory"] = [m for m in kb["spatial_memory"] if m["grid_idx"] != grid_idx]
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
        """
        
        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=prompt,
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema={
                        "type": "OBJECT",
                        "properties": {
                            "status": {"type": "STRING", "enum": ["FOUND_IN_MEMORY", "SEARCH_REQUESTED", "NOT_A_COMMAND"]},
                            "target_object": {"type": "STRING", "description": "The exact physical object isolated from the prompt"},
                            "coords": {"type": "ARRAY", "items": {"type": "INTEGER"}, "nullable": True},
                            "yaw": {"type": "NUMBER", "nullable": True},
                            "reasoning": {"type": "STRING"}
                        },
                        "required": ["status", "target_object", "reasoning"]
                    }
                )
            )
            
            result = json.loads(response.text)
            
            # Save the clean object name so the verification step can use it later
            if result.get("target_object"):
                self.last_target_object = result["target_object"]
            else:
                self.last_target_object = user_command
                
            return result
        except Exception as e:
            return {"status": "NOT_A_COMMAND", "coords": None, "reasoning": f"API Error: {e}"}

    def verify_target_presence(self, pil_image, target_name):
        """Phase 2: Validates if the target actually exists at the coordinates."""
        # Use the cleanly extracted object name instead of the raw user prompt
        actual_target = getattr(self, 'last_target_object', target_name)
        
        prompt = self.config['verification_prompt'].replace("{target}", actual_target)
        
        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[pil_image, prompt]
            )
            return "YES" in response.text.upper()
        except Exception:
            return False