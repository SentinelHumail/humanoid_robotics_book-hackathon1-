import logging
import os
from typing import Dict, Any, List, Optional
from config import OPENAI_API_KEY, TOGETHER_API_KEY, GROQ_API_KEY
from openai import OpenAI
import httpx

logger = logging.getLogger(__name__)


class RAGService:
    def __init__(self):
        logger.info(f"Groq API key present: {bool(GROQ_API_KEY)}")
        logger.info(f"Together API key present: {bool(TOGETHER_API_KEY)}")
        logger.info(f"OpenAI API key present: {bool(OPENAI_API_KEY)}")

        self.openai_client = None
        if OPENAI_API_KEY:
            self.openai_client = OpenAI(api_key=OPENAI_API_KEY)

    def generate_answer(
        self,
        user_query: str,
        context: str = "",
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate an answer using alternative AI services with fallback to OpenAI
        """
        # Prepare the prompt based on whether selected_text is provided
        if selected_text:
            prompt = f'The user has highlighted the following text from the book: {selected_text}. Based ONLY on this selection, answer the following: {user_query}'
        else:
            prompt = f"Context: {context}\n\nQuestion: {user_query}\n\nBased on the provided context, answer the question accurately. If the answer isn't in the context, politely say you don't know."

        # Define system message
        system_message = "You are an expert assistant for the book \"Physical AI & Humanoid Robotics\". Use the provided context to answer questions accurately. If the answer isn't in the context, politely say you don't know."

        # Try Groq first (Priority 1)
        if GROQ_API_KEY:
            logger.info("Attempting Groq API...")
            try:
                groq_response = self._call_groq_api(prompt, system_message)
                if groq_response:
                    logger.info("Groq API call successful")
                    return groq_response
            except Exception as e:
                logger.warning(f"Groq API failed with error: {str(e)}")
                logger.info("Groq failed, trying Together AI...")
        else:
            logger.info("Groq API key not configured, skipping Groq")

        # Try Together AI next (Priority 2)
        if TOGETHER_API_KEY:
            logger.info("Attempting Together API...")
            try:
                together_response = self._call_together_api(prompt, system_message)
                if together_response:
                    logger.info("Together API call successful")
                    return together_response
            except Exception as e:
                logger.warning(f"Together API failed with error: {str(e)}")
                logger.info("Together failed, trying OpenAI...")
        else:
            logger.info("Together API key not configured, skipping Together")

        # Fallback to OpenAI (Priority 3)
        if self.openai_client:
            logger.info("Attempting OpenAI API...")
            try:
                openai_response = self._call_openai_api(prompt, system_message)
                if openai_response:
                    logger.info("OpenAI API call successful")
                    return openai_response
            except Exception as e:
                logger.error(f"OpenAI API failed with error: {str(e)}")
                raise Exception(f"Unable to generate response from any AI service. All attempts failed: Groq, Together, OpenAI")
        else:
            logger.info("OpenAI API key not configured")

        raise Exception("No valid API keys configured for AI services. Please set at least one of: GROQ_API_KEY, TOGETHER_API_KEY, or OPENAI_API_KEY")

    def _call_groq_api(self, user_prompt: str, system_message: str) -> Optional[Dict[str, Any]]:
        """Call Groq API for completions"""
        headers = {
            "Authorization": f"Bearer {GROQ_API_KEY}",
            "Content-Type": "application/json"
        }

        data = {
            "model": "llama-3.1-8b-instant",
            "messages": [
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_prompt}
            ],
            "temperature": 0.3,
            "max_tokens": 1000
        }

        response = httpx.post(
            "https://api.groq.com/openai/v1/chat/completions",
            json=data,
            headers=headers,
            timeout=30.0
        )

        if response.status_code == 200:
            result = response.json()
            return {
                "answer": result["choices"][0]["message"]["content"],
                "model_used": "groq-llama-3.1-8b-instant"
            }
        else:
            raise Exception(f"Groq API error: {response.status_code} - {response.text}")

    def _call_together_api(self, user_prompt: str, system_message: str) -> Optional[Dict[str, Any]]:
        """Call Together AI API for completions"""
        headers = {
            "Authorization": f"Bearer {TOGETHER_API_KEY}",
            "Content-Type": "application/json"
        }

        data = {
            "model": "meta-llama/Meta-Llama-3.1-8B-Instruct-Turbo",
            "messages": [
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_prompt}
            ],
            "temperature": 0.3,
            "max_tokens": 1000
        }

        response = httpx.post(
            "https://api.together.xyz/v1/chat/completions",
            json=data,
            headers=headers,
            timeout=30.0
        )

        if response.status_code == 200:
            result = response.json()
            return {
                "answer": result["choices"][0]["message"]["content"],
                "model_used": "together-meta-llama/Meta-Llama-3.1-8B-Instruct-Turbo"
            }
        else:
            raise Exception(f"Together API error: {response.status_code} - {response.text}")

    def _call_openai_api(self, user_prompt: str, system_message: str) -> Optional[Dict[str, Any]]:
        """Call OpenAI API for completions (fallback)"""
        if not self.openai_client:
            raise Exception("OpenAI client not initialized")

        response = self.openai_client.chat.completions.create(
            model="gpt-4o",  # Using gpt-4o as requested originally
            messages=[
                {
                    "role": "system",
                    "content": system_message
                },
                {
                    "role": "user",
                    "content": user_prompt
                }
            ],
            max_tokens=1000,
            temperature=0.3
        )

        return {
            "answer": response.choices[0].message.content,
            "model_used": "openai-gpt-4o"
        }