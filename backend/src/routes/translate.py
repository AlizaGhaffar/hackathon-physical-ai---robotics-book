from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
import os
from openai import OpenAI

router = APIRouter(prefix="/api/translate", tags=["translate"])

# Initialize OpenAI client
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

class TranslateRequest(BaseModel):
    text: str
    target_language: str = "ur"  # Default to Urdu
    source_language: str = "en"  # Default from English

class TranslateResponse(BaseModel):
    translated_text: str
    source_language: str
    target_language: str

@router.post("", response_model=TranslateResponse)
async def translate_text(request: TranslateRequest):
    """
    Translate text from English to Urdu or vice versa
    """
    try:
        # Language mapping
        lang_names = {
            "ur": "Urdu",
            "en": "English",
        }

        target_lang_name = lang_names.get(request.target_language, "Urdu")
        source_lang_name = lang_names.get(request.source_language, "English")

        # Use OpenAI for translation
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": f"You are a professional translator. Translate the following text from {source_lang_name} to {target_lang_name}. Maintain the technical accuracy and context. Only provide the translation, no explanations."
                },
                {
                    "role": "user",
                    "content": request.text
                }
            ],
            temperature=0.3,
            max_tokens=1000
        )

        translated_text = response.choices[0].message.content.strip()

        return TranslateResponse(
            translated_text=translated_text,
            source_language=request.source_language,
            target_language=request.target_language
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Translation error: {str(e)}"
        )
