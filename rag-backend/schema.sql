-- User Authentication and Profile Schema for Neon Postgres

-- Users table for authentication
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    name VARCHAR(255),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- User profiles table for personalization
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    
    -- Software Background
    software_experience VARCHAR(50), -- 'beginner', 'intermediate', 'advanced'
    programming_languages TEXT[], -- Array of languages: ['Python', 'C++', 'JavaScript']
    has_ros_experience BOOLEAN DEFAULT FALSE,
    
    -- Hardware Background
    hardware_experience VARCHAR(50), -- 'none', 'basic', 'intermediate', 'advanced'
    hardware_description TEXT,
    has_robotics_hardware BOOLEAN DEFAULT FALSE,
    
    -- Learning Goals
    learning_goal VARCHAR(100), -- 'career_change', 'hobby', 'research', 'academic', 'professional'
    learning_goals_description TEXT,
    
    -- Preferences
    preferred_difficulty VARCHAR(50) DEFAULT 'intermediate',
    show_beginner_notes BOOLEAN DEFAULT TRUE,
    show_advanced_tips BOOLEAN DEFAULT FALSE,
    
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    
    UNIQUE(user_id)
);

-- Chat sessions table for tracking user interactions
CREATE TABLE IF NOT EXISTS chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    session_data JSONB,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Indexes for better performance
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
CREATE INDEX IF NOT EXISTS idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id ON chat_sessions(user_id);

-- Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Triggers for auto-updating updated_at
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_profiles_updated_at BEFORE UPDATE ON user_profiles
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
