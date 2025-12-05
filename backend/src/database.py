import psycopg
from .config import settings

def get_db_connection():
    """Get database connection"""
    return psycopg.connect(
        settings.neon_database_url,
        row_factory=psycopg.rows.dict_row
    )

def create_tables():
    """Create database tables if they don't exist"""
    conn = get_db_connection()
    cursor = conn.cursor()

    # Users table (T015)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS users (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            email VARCHAR(255) UNIQUE NOT NULL,
            password_hash VARCHAR(255) NOT NULL,
            name VARCHAR(100) NOT NULL,
            software_level VARCHAR(20) NOT NULL CHECK (software_level IN ('Beginner', 'Intermediate', 'Advanced')),
            hardware_level VARCHAR(20) NOT NULL CHECK (hardware_level IN ('Beginner', 'Intermediate', 'Advanced')),
            learning_goals TEXT,
            created_at TIMESTAMP DEFAULT NOW(),
            updated_at TIMESTAMP DEFAULT NOW()
        )
    """)

    # Progress records table (T016)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS progress_records (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
            chapter_id VARCHAR(50) NOT NULL,
            sections_completed TEXT[] DEFAULT '{}',
            quiz_score INTEGER CHECK (quiz_score >= 0 AND quiz_score <= 100),
            completion_date TIMESTAMP,
            updated_at TIMESTAMP DEFAULT NOW(),
            UNIQUE(user_id, chapter_id)
        )
    """)

    # Chat messages table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS chat_messages (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            session_id VARCHAR(100) NOT NULL,
            user_id UUID REFERENCES users(id) ON DELETE SET NULL,
            chapter_id VARCHAR(50) NOT NULL,
            user_question TEXT NOT NULL,
            bot_response TEXT NOT NULL,
            context_used JSONB,
            timestamp TIMESTAMP DEFAULT NOW()
        )
    """)

    # Translation cache table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS translation_cache (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            original_text_hash VARCHAR(64) UNIQUE NOT NULL,
            original_text TEXT NOT NULL,
            translated_text TEXT NOT NULL,
            language VARCHAR(10) NOT NULL,
            created_at TIMESTAMP DEFAULT NOW()
        )
    """)

    conn.commit()
    cursor.close()
    conn.close()
    print("[SUCCESS] Database tables created successfully")

if __name__ == "__main__":
    create_tables()
