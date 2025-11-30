# 🤖 Physical AI & Humanoid Robotics Textbook

[![Deploy to GitHub Pages](https://github.com/abdulrehman346790/panaversity-physical-ai-textbook/actions/workflows/deploy.yml/badge.svg)](https://github.com/abdulrehman346790/panaversity-physical-ai-textbook/actions/workflows/deploy.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Built with Docusaurus](https://img.shields.io/badge/Built%20with-Docusaurus-green.svg)](https://docusaurus.io/)

> **A comprehensive, open-source guide to Physical AI, Humanoid Robotics, and Intelligent Agents**

📚 **[Read the Book](https://abdulrehman346790.github.io/panaversity-physical-ai-textbook/)** | 🤖 **[Try the AI Chatbot](https://abdulrehman346790.github.io/panaversity-physical-ai-textbook/)** | ⭐ **[Star on GitHub](https://github.com/abdulrehman346790/panaversity-physical-ai-textbook)**

---

## ✨ Features

- 📖 **Comprehensive Curriculum**: From fundamentals to advanced topics in Physical AI
- 🤖 **AI-Powered Chatbot**: Ask questions and get instant answers powered by Gemini 2.5 Flash
- 🎨 **Modern Design**: Beautiful, responsive UI with dark mode support
- 🔧 **Hands-on Projects**: Practical code examples and exercises
- 📊 **Visual Diagrams**: Mermaid diagrams for complex concepts
- 🚀 **Production-Ready**: Deployed on GitHub Pages with CI/CD

## 📚 What You'll Learn

### Chapter 1: Introduction to Physical AI
- Foundations of Physical AI and embodied intelligence
- Humanoid robotics and real-world applications
- Sensors and actuators for perception and action

### Chapter 2: ROS 2 Fundamentals
- Robot Operating System 2 architecture
- Nodes, topics, services, and actions
- Python AI agent integration

### Chapter 3: Robot Simulation
- Webots and Gazebo simulation environments
- Creating virtual robots and controllers
- ROS 2 integration for testing

### Coming Soon
- 🧠 Chapter 4: Building an AI Brain
- 👁️ Chapter 5: Vision-Language-Action Models
- 🚀 Chapter 6: Deployment & Real-World Applications

## 🛠️ Tech Stack

### Frontend
- **Docusaurus** - Modern static site generator
- **React** - UI components
- **Mermaid** - Diagram rendering

### Backend (RAG Chatbot)
- **FastAPI** - High-performance Python web framework
- **OpenAI Agents SDK** - Agent orchestration
- **Gemini 2.5 Flash** - LLM for chat completions
- **Qdrant Cloud** - Vector database for embeddings
- **Neon Postgres** - Serverless database for session management

## 🚀 Quick Start

### Prerequisites
- Node.js 18+ and npm
- Python 3.10+
- Git

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/abdulrehman346790/panaversity-physical-ai-textbook.git
   cd panaversity-physical-ai-textbook
   ```

2. **Install Docusaurus dependencies**
   ```bash
   cd docusaurus/book
   npm install
   ```

3. **Run the development server**
   ```bash
   npm run start
   ```

4. **Open your browser**
   Navigate to `http://localhost:3000`

### Running the RAG Chatbot (Optional)

1. **Install Python dependencies**
   ```bash
   cd rag-backend
   pip install -r requirements.txt
   ```

2. **Create `.env` file**
   ```env
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   NEON_DATABASE_URL=your_neon_db_url
   ```

3. **Run the backend**
   ```bash
   uvicorn main:app --reload --port 8001
   ```

4. **Test the chatbot**
   ```powershell
   Invoke-RestMethod -Method Post -Uri "http://localhost:8001/chat" -ContentType "application/json" -Body '{"message": "What is Physical AI?"}'
   ```

## 📦 Deployment

This project is automatically deployed to GitHub Pages using GitHub Actions.

### Manual Deployment

```bash
cd docusaurus/book
npm run build
npm run serve
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Panaversity** - For the Physical AI curriculum
- **OpenAI** - For the Agents SDK
- **Google** - For Gemini API
- **Docusaurus** - For the amazing documentation framework

## 📧 Contact

Abdul Rehman - [@abdulrehman346790](https://github.com/abdulrehman346790)

Project Link: [https://github.com/abdulrehman346790/panaversity-physical-ai-textbook](https://github.com/abdulrehman346790/panaversity-physical-ai-textbook)

---

<p align="center">Made with ❤️ for the Physical AI community</p>
