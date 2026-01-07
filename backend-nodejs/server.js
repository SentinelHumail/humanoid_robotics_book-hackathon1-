const express = require('express');
const cors = require('cors');
const axios = require('axios');
const { Pool } = require('pg');
require('dotenv').config();

// Import OpenAI (for OpenRouter)
const { OpenAI } = require('openai');
// Import Google Generative Language API for embeddings
const { google } = require('googleapis');

// Import Qdrant
const { QdrantClient } = require('@qdrant/js-client-rest');

const app = express();
const PORT = process.env.PORT || 10000;  // Force port 10000

// Global logging middleware - logs every incoming request
app.use((req, res, next) => {
  console.log(req.method, req.url);
  next();
});

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

// Add GitHub Pages domains to the allowed origins
const githubPagesOrigins = [
  'https://SentinelHumail.github.io',
  'https://sentinelhumail.github.io'
];

const allOrigins = [...new Set([...allowedOrigins, ...defaultOrigins, ...githubPagesOrigins])];

app.use(cors({
  origin: allOrigins,
  credentials: true,
  methods: ['GET', 'POST', 'OPTIONS'],  // Add required methods
  allowedHeaders: ['Content-Type', 'Authorization'],  // Add authorization header as well
  optionsSuccessStatus: 200  // Some legacy browsers (IE11, various SmartTVs) choke on 204
}));

// Add OPTIONS route for all paths to handle preflight requests
app.options('*', cors());

// Initialize clients
let openaiClient = null;
let qdrantClient = null;
let ragService = null;

// Initialize clients
let openrouterClient = null;
let geminiEmbeddingsClient = null;

// Initialize OpenAI client with OpenRouter endpoint
if (process.env.OPENROUTER_API_KEY) {
  openrouterClient = new OpenAI({
    baseURL: "https://openrouter.ai/api/v1",
    apiKey: process.env.OPENROUTER_API_KEY
  });
} else {
  console.warn('Warning: OPENROUTER_API_KEY not set');
}

// Initialize Google Generative Language client for embeddings (using Gemini)
if (process.env.GEMINI_API_KEY) {
  // For now, we'll just store the API key for later use with Google APIs
  geminiEmbeddingsClient = {
    apiKey: process.env.GEMINI_API_KEY
  };
} else {
  console.warn('Warning: GEMINI_API_KEY not set for embeddings');
}

// For the rest of the code, we'll use openrouterClient for completions and geminiClient for embeddings
// Assign openrouterClient to openaiClient variable to maintain compatibility with the rest of the code
openaiClient = openrouterClient;

// Initialize Qdrant client
try {
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
} catch (error) {
  console.error('Error initializing Qdrant client:', error.message);
  console.warn('Continuing without Qdrant - search functionality may be limited');
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
  console.log('All services initialized');
  console.log('Groq client ready for completions');
  console.log('Embedding client ready for embeddings');
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
  console.log('API Request received at:', new Date().toISOString());
  try {
    const { user_query, user_id, selected_text } = req.body;

    if (!user_query || !user_id) {
      return res.status(400).json({ error: 'Missing required fields: user_query and user_id' });
    }

    // Search Qdrant for relevant chunks if no selected text
    let context = "";
    let sources = [];

    if (selected_text) {
      context = selected_text;
    } else if (user_query) {
      // Search Qdrant for relevant chunks
      if (!qdrantClient) {
        return res.status(500).json({ error: 'Qdrant service not available' });
      }

      if (!geminiEmbeddingsClient) {
        // If no embedding client is available, use OpenRouter directly for general questions
        // Generate a response using the OpenRouter client without document context
        if (!openrouterClient) {
          return res.status(500).json({ error: 'OpenRouter client not available' });
        }

        try {
          const response = await openrouterClient.chat.completions.create({
            model: "meta-llama/llama-3.3-70b-instruct:free",
            messages: [
              {
                role: "system",
                content: "You are an expert assistant for the book \"Physical AI & Humanoid Robotics\". Since document embeddings are not available, use your general knowledge to answer questions. If the question is very specific to the book content, politely acknowledge that you don't have access to the specific documents right now."
              },
              {
                role: "user",
                content: user_query
              }
            ],
            max_tokens: 1000,
            temperature: 0.3
          });

          const aiAnswer = response.choices[0].message.content;

          // Save the chat interaction to the database
          await saveChat(user_id, user_query, aiAnswer, []);

          // Return the response
          return res.json({
            answer: aiAnswer,
            sources: []
          });
        } catch (error) {
          console.error('Error generating response with OpenRouter:', error);
          return res.status(500).json({
            error: 'Error generating response',
            details: error.message
          });
        }
      }

      let searchResults = [];
      try {
        // For now, we'll implement a different approach since Gemini embeddings might require a different implementation
        // First, let's try using OpenAI-compatible embeddings from OpenRouter if they support it
        // Otherwise, we'll need to use a different embedding approach

        let queryEmbedding;

        // Try to use OpenAI-compatible embeddings from OpenRouter if they support it
        if (openrouterClient) {
          try {
            const embeddingResponse = await openrouterClient.embeddings.create({
              model: "text-embedding-3-small",
              input: user_query
            });

            queryEmbedding = embeddingResponse.data[0].embedding;
          } catch (embeddingError) {
            console.warn('OpenRouter embeddings not available, using fallback approach');
            // Use a fallback approach - for now, we'll return a general response
            const response = await openrouterClient.chat.completions.create({
              model: "meta-llama/llama-3.3-70b-instruct:free",
              messages: [
                {
                  role: "system",
                  content: "You are an expert assistant for the book \"Physical AI & Humanoid Robotics\". Document search is not available at the moment, so use your general knowledge to answer questions. If the question is very specific to the book content, politely acknowledge that you don't have access to the specific documents right now."
                },
                {
                  role: "user",
                  content: user_query
                }
              ],
              max_tokens: 1000,
              temperature: 0.3
            });

            const aiAnswer = response.choices[0].message.content;

            // Save the chat interaction to the database
            await saveChat(user_id, user_query, aiAnswer, []);

            // Return the response
            return res.json({
              answer: aiAnswer,
              sources: []
            });
          }
        } else {
          // If no embedding service is available, return a general response
          const response = await openrouterClient.chat.completions.create({
            model: "meta-llama/llama-3.3-70b-instruct:free",
            messages: [
              {
                role: "system",
                content: "You are an expert assistant for the book \"Physical AI & Humanoid Robotics\". Document search is not available at the moment, so use your general knowledge to answer questions. If the question is very specific to the book content, politely acknowledge that you don't have access to the specific documents right now."
              },
              {
                role: "user",
                content: user_query
              }
            ],
            max_tokens: 1000,
            temperature: 0.3
          });

          const aiAnswer = response.choices[0].message.content;

          // Save the chat interaction to the database
          await saveChat(user_id, user_query, aiAnswer, []);

          // Return the response
          return res.json({
            answer: aiAnswer,
            sources: []
          });
        }

        // If we successfully got embeddings, continue with Qdrant search
        // Search Qdrant for top 3 most relevant chunks
        searchResults = await qdrantClient.search('book_embeddings', {
          vector: queryEmbedding,
          limit: 3,
          with_payload: true
        });
      } catch (error) {
        console.error('Error searching Qdrant:', error);
        return res.status(500).json({ error: 'Error searching document database', details: error.message });
      }

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
    }

    // Generate the response using the OpenRouter client
    if (!openrouterClient) {
      return res.status(500).json({ error: 'OpenRouter client not available' });
    }

    const response = await openrouterClient.chat.completions.create({
      model: "meta-llama/llama-3.3-70b-instruct:free", // Using OpenRouter's free model
      messages: [
        {
          role: "system",
          content: "You are an expert assistant for the book \"Physical AI & Humanoid Robotics\". Use the provided context to answer questions accurately. If the answer isn't in the context, politely say you don't know."
        },
        {
          role: "user",
          content: `Context:\n${context}\n\nQuestion: ${user_query}`
        }
      ],
      max_tokens: 1000,
      temperature: 0.3
    });

    const aiAnswer = response.choices[0].message.content;

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
      error: error.message,
      stack: error.stack
    });
  }
});

// Test route
app.get('/test', (req, res) => {
  res.send('Backend is reachable');
});

// Start server - force port 10000 and bind to 0.0.0.0
app.listen(process.env.PORT || 10000, '0.0.0.0', () => {
  console.log(`Server running on 0.0.0.0:${process.env.PORT || 10000}`);
});

// Global error handler
app.use((err, req, res, next) => {
  console.error('SERVER CRASH:', err.stack);
  res.status(500).json({ error: 'Server Crash', detail: err.message });
});

module.exports = app;