const express = require('express');
const cors = require('cors');
const axios = require('axios');
const { Pool } = require('pg');
require('dotenv').config();

// Import OpenAI
const { OpenAI } = require('openai');

// Import Qdrant
const { QdrantClient } = require('qdrant-client');

// Import RAG Service
const RAGService = require('./services/ragService');

const app = express();
const PORT = process.env.PORT || 8000;

// Middleware
app.use(express.json());

// Configure CORS with flexible origins for deployment
const allowedOrigins = process.env.ALLOWED_ORIGINS
  ? process.env.ALLOWED_ORIGINS.split(',').map(origin => origin.trim())
  : ['http://localhost:3000'];

const defaultOrigins = [
  'http://localhost:3000',
  'http://localhost:3001',
  'http://localhost:8080',
  'http://localhost:5173',
  'http://localhost:5174',
  'https://127.0.0.1:3000',
  'https://127.0.0.1:8080',
  'https://localhost:3000',
  'https://localhost:8080'
];

const allOrigins = [...new Set([...allowedOrigins, ...defaultOrigins])];

app.use(cors({
  origin: allOrigins,
  credentials: true
}));

// Initialize clients
let openaiClient = null;
let qdrantClient = null;
let ragService = null;

// Initialize OpenAI client
if (process.env.OPENAI_API_KEY) {
  openaiClient = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });
} else {
  console.warn('Warning: OPENAI_API_KEY not set');
}

// Initialize Qdrant client
if (process.env.QDRANT_URL) {
  const qdrantApiKey = process.env.QDRANT_API_KEY;
  if (qdrantApiKey) {
    qdrantClient = new QdrantClient({
      url: process.env.QDRANT_URL,
      apiKey: qdrantApiKey
    });
  } else {
    qdrantClient = new QdrantClient({
      url: process.env.QDRANT_URL
    });
  }
} else {
  console.warn('Warning: QDRANT_URL not set');
}

// Initialize PostgreSQL pool
const pool = new Pool({
  connectionString: process.env.NEON_DATABASE_URL,
  ssl: process.env.NODE_ENV === 'production' ? { rejectUnauthorized: false } : false
});

// Test database connection and create tables if they don't exist
async function initDatabase() {
  try {
    const client = await pool.connect();

    // Create chat_history table
    await client.query(`
      CREATE TABLE IF NOT EXISTS chat_history (
        id SERIAL PRIMARY KEY,
        user_id VARCHAR(255),
        question TEXT,
        answer TEXT,
        sources TEXT,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
      );
    `);

    // Create book_chunks table
    await client.query(`
      CREATE TABLE IF NOT EXISTS book_chunks (
        id SERIAL PRIMARY KEY,
        content TEXT,
        source_file VARCHAR(500),
        chapter VARCHAR(255),
        chunk_index INTEGER,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
      );
    `);

    console.log('Database tables initialized');
    client.release();
  } catch (err) {
    console.error('Error initializing database:', err);
  }
}

// Initialize Qdrant collection
async function initQdrant() {
  if (!qdrantClient) {
    console.warn('Qdrant client not available, skipping initialization');
    return;
  }

  try {
    const collectionName = 'book_embeddings';

    // Check if collection exists
    try {
      await qdrantClient.getCollection(collectionName);
      console.log('Qdrant collection already exists');
    } catch (err) {
      // Collection doesn't exist, create it
      await qdrantClient.createCollection(collectionName, {
        vectors: {
          size: 1536, // Standard size for OpenAI embeddings
          distance: 'Cosine'
        }
      });
      console.log('Qdrant collection created');
    }
  } catch (err) {
    console.error('Error initializing Qdrant:', err);
  }
}

// Initialize services on startup
async function initializeServices() {
  await initDatabase();
  await initQdrant();
  ragService = new RAGService(); // Initialize the RAG service
  console.log('All services initialized');
}

initializeServices();

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({
    status: 'ok',
    message: 'RAG Chatbot API'
  });
});

// DB status endpoint
app.get('/db-status', async (req, res) => {
  let dbConnected = false;
  let qdrantConnected = false;

  // Test database connection
  try {
    const client = await pool.connect();
    await client.query('SELECT 1');
    dbConnected = true;
    client.release();
  } catch (err) {
    console.error('Database connection error:', err);
  }

  // Test Qdrant connection
  try {
    if (qdrantClient) {
      await qdrantClient.getCollections();
      qdrantConnected = true;
    }
  } catch (err) {
    console.error('Qdrant connection error:', err);
  }

  res.json({
    status: 'ok',
    database_connected: dbConnected,
    qdrant_connected: qdrantConnected,
    message: 'Database and Qdrant connection status'
  });
});

// Function to save chat history
async function saveChat(userId, question, answer, sources) {
  try {
    await pool.query(
      'INSERT INTO chat_history (user_id, question, answer, sources) VALUES ($1, $2, $3, $4)',
      [userId, question, answer, JSON.stringify(sources)]
    );
  } catch (err) {
    console.error('Error saving chat:', err);
  }
}

// Function to get chat history
async function getChatHistory(userId, limit = 10) {
  try {
    const result = await pool.query(
      'SELECT * FROM chat_history WHERE user_id = $1 ORDER BY created_at DESC LIMIT $2',
      [userId, limit]
    );
    return result.rows;
  } catch (err) {
    console.error('Error getting chat history:', err);
    return [];
  }
}

// Chat endpoint
app.post('/chat', async (req, res) => {
  try {
    const { user_query, user_id, selected_text } = req.body;

    if (!user_query || !user_id) {
      return res.status(400).json({ error: 'Missing required fields: user_query and user_id' });
    }

    // Search Qdrant for relevant chunks if no selected text
    let context = "";
    let sources = [];

    if (!selected_text) {
      // Search Qdrant for relevant chunks
      if (!qdrantClient || !openaiClient) {
        return res.status(500).json({ error: 'Required services (Qdrant/OpenAI) not available' });
      }

      // Generate embedding for the user query
      const embeddingResponse = await openaiClient.embeddings.create({
        model: "text-embedding-3-small",
        input: user_query
      });

      const queryEmbedding = embeddingResponse.data[0].embedding;

      // Search Qdrant for top 3 most relevant chunks
      const searchResults = await qdrantClient.search('book_embeddings', {
        vector: queryEmbedding,
        limit: 3,
        with_payload: true
      });

      // Build context from search results
      const contextParts = [];

      searchResults.forEach(result => {
        const payload = result.payload;
        const content = payload?.content || '';
        const sourceFile = payload?.source_file || 'unknown';

        contextParts.push(content);
        sources.push({
          source_file: sourceFile,
          content: content.length > 200 ? content.substring(0, 200) + "..." : content,
          score: result.score
        });
      });

      // Combine the context
      context = contextParts.join('\n\n');
    } else {
      // Use selected text as context
      sources = [{ source_file: "selected_text", content: selected_text }];
    }

    // Use the RAG service to generate the answer with fallback options
    const responseData = await ragService.generateAnswer(
      user_query,
      context,
      selected_text
    );

    const aiAnswer = responseData.answer;

    // Save the chat interaction to the database
    await saveChat(user_id, user_query, aiAnswer, sources.map(source => source.source_file));

    // Return the response
    res.json({
      answer: aiAnswer,
      sources: sources
    });

  } catch (error) {
    console.error('Error in chat endpoint:', error);
    res.status(500).json({
      error: 'Error processing chat request',
      message: error.message
    });
  }
});

// Start server
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

module.exports = app;