from database import engine, Base, get_db
from sqlalchemy import text
import os

def init_db():
    print("Initializing database schema...")
    
    # Read schema.sql
    with open("schema.sql", "r") as f:
        schema_sql = f.read()
    
    # Execute schema
    with engine.connect() as connection:
        # Split by statements (simple split by ;)
        statements = schema_sql.split(';')
        for statement in statements:
            if statement.strip():
                try:
                    connection.execute(text(statement))
                    connection.commit()
                except Exception as e:
                    print(f"Error executing statement: {e}")
                    connection.rollback()
    
    print("Database schema initialized successfully!")

if __name__ == "__main__":
    init_db()
