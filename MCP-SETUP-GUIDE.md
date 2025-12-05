# 🚀 MCP Server Setup Guide

## ✅ Configuration File Created!

Location: `.claude/mcp-settings.json`

## 📋 Step-by-Step Setup Instructions:

### **Step 1: Get Context 7 API Key**

Context 7 requires an API key from Upstash:

1. Visit: https://console.upstash.com/
2. Sign up / Log in
3. Create a new Context 7 instance
4. Copy your API key

### **Step 2: Update Configuration File**

Open `.claude/mcp-settings.json` and replace:
- `"your-context7-api-key-here"` with your actual Context 7 API key
- Optionally add GitHub Personal Access Token (if needed)

### **Step 3: Configure in Claude Code**

#### **Method A: Through Claude Code Settings UI**

1. Open Claude Code application
2. Click on **Settings** (⚙️ icon)
3. Look for **"MCP Servers"** or **"Model Context Protocol"** section
4. Click **"Import from file"** or **"Add MCP Configuration"**
5. Select the file: `.claude/mcp-settings.json`
6. Save and restart Claude Code

#### **Method B: Manual Configuration File (if Method A doesn't work)**

Copy the `.claude/mcp-settings.json` content to one of these locations:

**Windows:**
```
%APPDATA%\Claude\claude_desktop_config.json
```
OR
```
C:\Users\affil\.config\claude-code\config.json
```

**Create the directory if it doesn't exist:**
```bash
mkdir -p C:\Users\affil\.config\claude-code
```

### **Step 4: Restart Claude Code**

After configuration, **completely restart** Claude Code application.

### **Step 5: Verify MCP Servers are Connected**

In a new conversation, you can check if MCP tools are available by asking Claude to list available MCP tools.

## 📦 Configured MCP Servers:

### 1. **Context 7** (@upstash/context7-mcp)
- **Purpose:** Up-to-date documentation and code context
- **Useful for:** Getting latest library docs, API references
- **Requires:** API key from Upstash

### 2. **Filesystem** (@modelcontextprotocol/server-filesystem)
- **Purpose:** Enhanced file operations
- **Useful for:** Better file management
- **Requires:** No API key

### 3. **GitHub** (@modelcontextprotocol/server-github)
- **Purpose:** GitHub repository operations
- **Useful for:** Repo management, issues, PRs
- **Requires:** GitHub Personal Access Token (optional)

## 🔧 Troubleshooting:

### Issue: "MCP servers not connecting"
**Solution:**
- Check if Node.js is installed: `node --version`
- Ensure npm is working: `npm --version`
- Verify API keys are correct
- Restart Claude Code completely

### Issue: "Context 7 authentication failed"
**Solution:**
- Verify API key from Upstash Console
- Check for extra spaces in the API key
- Ensure you're using the correct Context 7 instance

### Issue: "Cannot find configuration file"
**Solution:**
- Create the directory manually
- Check file permissions
- Use absolute paths in configuration

## 📞 Need Help?

If MCP servers still don't work after following these steps, you can:
1. Continue working without MCP servers (regular tools work fine)
2. Check Claude Code documentation
3. Ask me for alternative approaches

## ⚡ Quick Test Commands:

Once MCP is set up, test with:
- "List available MCP tools"
- "Use Context 7 to search for Docusaurus documentation"
- "Show me filesystem MCP capabilities"

---

**Note:** MCP servers are optional but enhance capabilities. The hackathon project can be completed with or without MCP servers!
