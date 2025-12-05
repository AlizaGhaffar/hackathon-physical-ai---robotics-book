from openai import OpenAI
from .config import settings

def get_openai_client():
    """Get OpenAI client"""
    return OpenAI(api_key=settings.openai_api_key)

def generate_embedding(text: str):
    """Generate embedding for text"""
    client = get_openai_client()
    response = client.embeddings.create(
        model="text-embedding-ada-002",
        input=text
    )
    return response.data[0].embedding

def translate_to_urdu(text: str):
    """Translate text to Urdu"""
    client = get_openai_client()
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a technical translator. Translate to Urdu. PRESERVE these terms in English: ROS 2, Node, Topic, Service, Publisher, Subscriber, rclpy."},
            {"role": "user", "content": f"Translate: {text}"}
        ],
        temperature=0.3
    )
    return response.choices[0].message.content

def chat_completion(prompt: str, context: str):
    """Generate chatbot response"""
    client = get_openai_client()
    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": f"Answer based on this context:\n{context}"},
            {"role": "user", "content": prompt}
        ],
        temperature=0.7
    )
    return response.choices[0].message.content
