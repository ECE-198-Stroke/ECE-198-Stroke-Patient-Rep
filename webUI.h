/*
 * webUI.h - the page served at "/". It polls /api/status and calls /api/start and /api/reset.
 */
#pragma once

static const char WEB_UI[] = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Table slide monitor</title>
<style>
:root{--bg:#f6f5f1;--card:#fff;--text:#1f1e1b;--muted:#6b6a64;--line:#e3e1d8;--accent:#0f6e56;--accent-text:#fff;--warn:#854f0b;--warn-bg:#faeeda;--go:#0f6e56;--go-bg:#e1f5ee;--idle-bg:#eeede6}
@media (prefers-color-scheme:dark){:root{--bg:#1b1b19;--card:#252523;--text:#ecebe5;--muted:#a3a29b;--line:#3a3935;--accent:#5dcaa5;--accent-text:#04342c;--warn:#fac775;--warn-bg:#412402;--go:#9fe1cb;--go-bg:#04342c;--idle-bg:#2c2c2a}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:16px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:720px;margin:0 auto;padding:20px 16px 40px}
h1{font-size:22px;font-weight:600;margin:0 0 4px}
.sub{color:var(--muted);font-size:14px;margin:0 0 20px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:16px}
.row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
.pill{display:inline-block;padding:6px 12px;border-radius:999px;font-weight:600;background:var(--idle-bg);color:var(--muted)}
.pill.calibrating{background:var(--warn-bg);color:var(--warn)}
.pill.ready{background:var(--go-bg);color:var(--go)}
.msg{color:var(--muted);font-size:14px;margin-top:8px;min-height:21px}
button{font:inherit;border-radius:10px;border:1px solid var(--line);padding:12px 18px;cursor:pointer;background:var(--card);color:var(--text)}
button.primary{background:var(--accent);color:var(--accent-text);border-color:var(--accent);font-weight:600;font-size:18px;padding:14px 28px}
button:disabled{opacity:.5;cursor:default}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px}
.stat .label{color:var(--muted);font-size:13px}
.stat .value{font-size:24px;font-weight:600}
.stat .unit{font-size:14px;color:var(--muted);font-weight:400}
h2{font-size:16px;font-weight:600;margin:0 0 12px}
table{width:100%;border-collapse:collapse;font-size:14px}
th,td{text-align:right;padding:6px 8px;border-bottom:1px solid var(--line)}
th:first-child,td:first-child{text-align:left}
th{color:var(--muted);font-weight:500}
.empty{color:var(--muted);font-size:14px}
.offline{color:var(--warn);font-size:14px;margin-left:auto}
.note{color:var(--muted);font-size:13px;margin-top:10px}
</style>
</head>
<body>
<main>
<h1>Table slide monitor</h1>
<p class="sub">Start a rep here or with the button. Results update live.</p>

<div class="card">
  <div class="row">
    <span id="state" class="pill">Connecting</span>
    <span id="offline" class="offline"></span>
  </div>
  <div id="message" class="msg"></div>
  <div class="row" style="margin-top:12px">
    <button id="start" class="primary">Start rep</button>
  </div>
</div>

<div class="card">
  <h2>Last rep</h2>
  <div id="last" class="stats"><div class="empty">No reps yet.</div></div>
</div>

<div class="card">
  <div class="row" style="justify-content:space-between;margin-bottom:12px">
    <h2 style="margin:0">Session</h2>
    <button id="reset">Reset session</button>
  </div>
  <div id="session" class="stats"></div>
  <div style="overflow-x:auto;margin-top:16px">
    <table>
      <thead><tr><th>Rep</th><th>Time (s)</th><th>Smoothness (jerk)</th><th>Est. distance (cm)</th><th>Peak (g)</th></tr></thead>
      <tbody id="history"></tbody>
    </table>
  </div>
  <p class="note">Lower jerk means smoother. Distance is an estimate and needs a stop after the rep.</p>
</div>
</main>
<script>
const STATE_TEXT={idle:"Idle",calibrating:"Keep the sled still",ready:"Ready, slide now"};
const fmt=(v,d)=>v===null||v===undefined?"n/a":Number(v).toFixed(d);
function stat(label,value,unit){const u=unit&&value!=="n/a"?` <span class="unit">${unit}</span>`:"";return `<div class="stat"><div class="label">${label}</div><div class="value">${value}${u}</div></div>`}
function render(s){
  const pill=document.getElementById("state");
  pill.textContent=STATE_TEXT[s.state]||s.state;
  pill.className="pill "+s.state;
  document.getElementById("message").textContent=s.message||"";
  document.getElementById("start").textContent=s.state==="idle"?"Start rep":"Restart rep";
  const last=s.history[0];
  document.getElementById("last").innerHTML=last
    ?stat("Rep",last.n)+stat("Time",fmt(last.time,2),"s")+stat("Smoothness",fmt(last.jerk,2),"jerk")+stat("Est. distance",fmt(last.dist,0),"cm")+stat("Peak",fmt(last.peak,3),"g")
    :`<div class="empty">No reps yet.</div>`;
  const ss=s.session;
  document.getElementById("session").innerHTML=stat("Reps",ss.reps)+stat("Mean time",fmt(ss.meanTime,2),"s")+stat("Mean jerk",fmt(ss.meanJerk,2))+stat("Mean distance",fmt(ss.meanDist,0),"cm");
  document.getElementById("history").innerHTML=s.history.length
    ?s.history.map(h=>`<tr><td>${h.n}</td><td>${fmt(h.time,2)}</td><td>${fmt(h.jerk,2)}</td><td>${fmt(h.dist,0)}</td><td>${fmt(h.peak,3)}</td></tr>`).join("")
    :`<tr><td colspan="5" class="empty">No reps yet.</td></tr>`;
}
async function poll(){
  try{
    const r=await fetch("/api/status",{cache:"no-store"});
    render(await r.json());
    document.getElementById("offline").textContent="";
  }catch(e){
    document.getElementById("offline").textContent="Lost connection to the device";
  }
}
async function post(path,button){
  button.disabled=true;
  try{await fetch(path,{method:"POST"});}catch(e){}
  button.disabled=false;
  poll();
}
document.getElementById("start").onclick=e=>post("/api/start",e.currentTarget);
document.getElementById("reset").onclick=e=>{if(confirm("Clear all reps in this session?"))post("/api/reset",e.currentTarget)};
poll();
setInterval(poll,500);
</script>
</body>
</html>
)HTML";
