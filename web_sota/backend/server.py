import sys
from contextlib import asynccontextmanager
from pathlib import Path

import httpx
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield


app = FastAPI(title="robotics-mcp", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.get("/api/llm/providers")
async def llm_providers():
    providers = []
    try:
        async with httpx.AsyncClient(timeout=3.0) as client:
            resp = await client.get("http://127.0.0.1:11434/api/tags")
            if resp.status_code == 200:
                data = resp.json()
                models = [m["name"] for m in data.get("models", [])]
                providers.append({"id": "ollama", "label": "Ollama", "base_url": "http://127.0.0.1:11434/v1", "models": models, "needs_key": False})
    except Exception:
        providers.append({"id": "ollama", "label": "Ollama", "base_url": "http://127.0.0.1:11434/v1", "models": [], "needs_key": False})
    try:
        async with httpx.AsyncClient(timeout=3.0) as client:
            resp = await client.get("http://127.0.0.1:1234/v1/models")
            if resp.status_code == 200:
                data = resp.json()
                models = [m["id"] for m in data.get("data", [])]
                providers.append({"id": "lmstudio", "label": "LM Studio", "base_url": "http://127.0.0.1:1234/v1", "models": models, "needs_key": False})
    except Exception:
        providers.append({"id": "lmstudio", "label": "LM Studio", "base_url": "http://127.0.0.1:1234/v1", "models": [], "needs_key": False})
    return {"providers": providers}


@app.post("/api/llm/chat")
async def llm_chat(body: dict):
    provider = body.get("provider", "ollama")
    model = body.get("model", "llama3.2:3b")
    prompt = body.get("prompt") or body.get("message", "")
    base = "http://127.0.0.1:1234/v1" if provider == "lmstudio" else "http://127.0.0.1:11434/v1"
    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            resp = await client.post(f"{base}/chat/completions", json={
                "model": model,
                "messages": [{"role": "user", "content": prompt}],
            })
            if resp.status_code == 200:
                data = resp.json()
                return {"response": data["choices"][0]["message"]["content"]}
            return {"response": f"HTTP {resp.status_code}"}
    except Exception as e:
        return {"response": f"Error: {e}"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
