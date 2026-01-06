const axios = require('axios');
require('dotenv').config();

class RAGService {
  constructor() {
    this.openaiApiKey = process.env.OPENAI_API_KEY;
    this.togetherApiKey = process.env.TOGETHER_API_KEY;
    this.groqApiKey = process.env.GROQ_API_KEY;

    console.log(`Groq API key present: ${!!this.groqApiKey}`);
    console.log(`Together API key present: ${!!this.togetherApiKey}`);
    console.log(`OpenAI API key present: ${!!this.openaiApiKey}`);
  }

  async generateAnswer(userQuery, context = "", selectedText = null) {
    // Prepare the prompt based on whether selected_text is provided
    let prompt;
    let sources = [];

    if (selectedText) {
      prompt = `The user has highlighted the following text from the book: ${selectedText}. Based ONLY on this selection, answer the following: ${userQuery}`;
      sources = [{ source_file: "selected_text", content: selectedText }];
    } else {
      prompt = `Context: ${context}\n\nQuestion: ${userQuery}\n\nBased on the provided context, answer the question accurately. If the answer isn't in the context, politely say you don't know.`;
    }

    // Define system message
    const systemMessage = "You are an expert assistant for the book \"Physical AI & Humanoid Robotics\". Use the provided context to answer questions accurately. If the answer isn't in the context, politely say you don't know.";

    // Try Groq first (Priority 1)
    if (this.groqApiKey) {
      console.log("Attempting Groq API...");
      try {
        const groqResponse = await this._callGroqApi(prompt, systemMessage);
        if (groqResponse) {
          console.log("Groq API call successful");
          return groqResponse;
        }
      } catch (error) {
        console.warn(`Groq API failed with error: ${error.message}`);
        console.log("Groq failed, trying Together AI...");
      }
    } else {
      console.log("Groq API key not configured, skipping Groq");
    }

    // Try Together AI next (Priority 2)
    if (this.togetherApiKey) {
      console.log("Attempting Together API...");
      try {
        const togetherResponse = await this._callTogetherApi(prompt, systemMessage);
        if (togetherResponse) {
          console.log("Together API call successful");
          return togetherResponse;
        }
      } catch (error) {
        console.warn(`Together API failed with error: ${error.message}`);
        console.log("Together failed, trying OpenAI...");
      }
    } else {
      console.log("Together API key not configured, skipping Together");
    }

    // Fallback to OpenAI (Priority 3)
    if (this.openaiApiKey) {
      console.log("Attempting OpenAI API...");
      try {
        const openaiResponse = await this._callOpenaiApi(prompt, systemMessage);
        if (openaiResponse) {
          console.log("OpenAI API call successful");
          return openaiResponse;
        }
      } catch (error) {
        console.error(`OpenAI API failed with error: ${error.message}`);
        throw new Error("Unable to generate response from any AI service. All attempts failed: Groq, Together, OpenAI");
      }
    } else {
      console.log("OpenAI API key not configured");
    }

    throw new Error("No valid API keys configured for AI services. Please set at least one of: GROQ_API_KEY, TOGETHER_API_KEY, or OPENAI_API_KEY");
  }

  async _callGroqApi(userPrompt, systemMessage) {
    const headers = {
      'Authorization': `Bearer ${this.groqApiKey}`,
      'Content-Type': 'application/json'
    };

    const data = {
      model: "llama-3.1-8b-instant",
      messages: [
        { role: "system", content: systemMessage },
        { role: "user", content: userPrompt }
      ],
      temperature: 0.3,
      max_tokens: 1000
    };

    const response = await axios.post(
      'https://api.groq.com/openai/v1/chat/completions',
      data,
      { headers, timeout: 30000 }
    );

    if (response.status === 200) {
      return {
        answer: response.data.choices[0].message.content,
        model_used: "groq-llama-3.1-8b-instant"
      };
    } else {
      throw new Error(`Groq API error: ${response.status} - ${response.statusText}`);
    }
  }

  async _callTogetherApi(userPrompt, systemMessage) {
    const headers = {
      'Authorization': `Bearer ${this.togetherApiKey}`,
      'Content-Type': 'application/json'
    };

    const data = {
      model: "meta-llama/Meta-Llama-3.1-8B-Instruct-Turbo",
      messages: [
        { role: "system", content: systemMessage },
        { role: "user", content: userPrompt }
      ],
      temperature: 0.3,
      max_tokens: 1000
    };

    const response = await axios.post(
      'https://api.together.xyz/v1/chat/completions',
      data,
      { headers, timeout: 30000 }
    );

    if (response.status === 200) {
      return {
        answer: response.data.choices[0].message.content,
        model_used: "together-meta-llama/Meta-Llama-3.1-8B-Instruct-Turbo"
      };
    } else {
      throw new Error(`Together API error: ${response.status} - ${response.statusText}`);
    }
  }

  async _callOpenaiApi(userPrompt, systemMessage) {
    if (!this.openaiApiKey) {
      throw new Error("OpenAI API key not configured");
    }

    const axios = require('axios');

    const headers = {
      'Authorization': `Bearer ${this.openaiApiKey}`,
      'Content-Type': 'application/json'
    };

    const data = {
      model: "gpt-4o",
      messages: [
        { role: "system", content: systemMessage },
        { role: "user", content: userPrompt }
      ],
      max_tokens: 1000,
      temperature: 0.3
    };

    const response = await axios.post(
      'https://api.openai.com/v1/chat/completions',
      data,
      { headers, timeout: 30000 }
    );

    if (response.status === 200) {
      return {
        answer: response.data.choices[0].message.content,
        model_used: "openai-gpt-4o"
      };
    } else {
      throw new Error(`OpenAI API error: ${response.status} - ${response.statusText}`);
    }
  }
}

module.exports = RAGService;