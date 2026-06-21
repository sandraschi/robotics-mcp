import { useState, useRef, useEffect } from "react";
import { API_BASE } from "@/lib/api";

interface Message {
  role: "user" | "assistant";
  content: string;
}

export default function FloatingChat() {
  const [open, setOpen] = useState(false);
  const [chat, setChat] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const [provider, setProvider] = useState(() => localStorage.getItem("llm_provider") || "ollama");
  const [model, setModel] = useState(() => localStorage.getItem("llm_model") || "");
  const [modelList, setModelList] = useState<string[]>([]);
  const bottomRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    fetch(`${API_BASE}/api/llm/providers`)
      .then((r) => r.json())
      .then((d) => {
        const providers = d.providers || d;
        const list: string[] = [];
        if (Array.isArray(providers)) {
          for (const p of providers) {
            if (p.models) list.push(...p.models);
          }
        }
        setModelList(list);
        if (!model && list.length > 0) {
          setModel(list[0]);
          localStorage.setItem("llm_model", list[0]);
        }
      })
      .catch(() => {});
  }, []);

  useEffect(() => {
    if (open) bottomRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [chat, open]);

  const sendMessage = async (text: string) => {
    setChat((prev) => [...prev, { role: "user", content: text }]);
    setLoading(true);
    try {
      const r = await fetch(`${API_BASE}/api/llm/chat`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ provider, model, prompt: text }),
      });
      const data = await r.json();
      setChat((prev) => [...prev, { role: "assistant", content: data.response || data.error || "No response" }]);
    } catch {
      setChat((prev) => [...prev, { role: "assistant", content: "Request failed. Is the backend running?" }]);
    }
    setLoading(false);
  };

  const handleSend = () => {
    if (!input.trim()) return;
    sendMessage(input.trim());
    setInput("");
  };

  return (
    <div className="fixed bottom-5 right-5 z-50">
      {open ? (
        <div className="bg-slate-900 border border-slate-700 rounded-2xl shadow-2xl w-[380px] h-[520px] flex flex-col overflow-hidden animate-in fade-in zoom-in-95 duration-200">
          <div className="flex items-center justify-between px-4 py-3 border-b border-slate-700">
            <span className="text-sm font-semibold text-slate-200">Chat</span>
            <div className="flex items-center gap-2">
              {modelList.length > 0 && (
                <select
                  className="bg-slate-800 border border-slate-600 rounded text-xs px-2 py-1 text-slate-300 max-w-[140px]"
                  value={model}
                  onChange={(e) => { setModel(e.target.value); localStorage.setItem("llm_model", e.target.value); }}
                >
                  {modelList.map((m) => <option key={m} value={m}>{m.split(":")[0]}</option>)}
                </select>
              )}
              <button onClick={() => setOpen(false)} className="text-slate-500 hover:text-slate-300 text-lg leading-none">&times;</button>
            </div>
          </div>
          <div className="flex-1 overflow-y-auto p-3 space-y-2 text-sm">
            {chat.length === 0 && (
              <p className="text-slate-500 text-xs text-center pt-8">Ask a question about this simulation.</p>
            )}
            {chat.map((msg, i) => (
              <div key={i} className={`flex ${msg.role === "user" ? "justify-end" : "justify-start"}`}>
                <div className={`max-w-[85%] rounded-xl px-3 py-2 whitespace-pre-wrap ${
                  msg.role === "user" ? "bg-cyan-800 text-cyan-100" : "bg-slate-800 text-slate-300"
                }`}>
                  {msg.content}
                </div>
              </div>
            ))}
            {loading && <div className="text-slate-500 text-xs animate-pulse">Thinking...</div>}
            <div ref={bottomRef} />
          </div>
          <div className="border-t border-slate-700 p-3 flex gap-2">
            <input
              className="flex-1 bg-slate-800 border border-slate-600 rounded-lg px-3 py-2 text-sm text-slate-200 focus:outline-none focus:border-cyan-500"
              placeholder="Ask something..."
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={(e) => e.key === "Enter" && handleSend()}
            />
            <button
              onClick={handleSend}
              disabled={loading || !input.trim()}
              className="bg-cyan-700 hover:bg-cyan-600 disabled:bg-slate-700 text-white px-3 py-2 rounded-lg text-sm font-medium"
            >
              Go
            </button>
          </div>
        </div>
      ) : (
        <button
          onClick={() => setOpen(true)}
          className="h-12 w-12 rounded-full bg-cyan-700 hover:bg-cyan-600 shadow-xl flex items-center justify-center text-white text-xl transition-colors"
          title="Open chat"
        >
          <svg xmlns="http://www.w3.org/2000/svg" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      )}
    </div>
  );
}
