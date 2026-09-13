"use strict";
const names=["table_left","table_right"];
const labels={table_left:"198 / left / action[0]",table_right:"66 / right / action[1]"};
const shortLabels={table_left:"198 / left",table_right:"66 / right"};
const colors={table_left:"#137e74",table_right:"#b97020"};
const indexes={table_left:0,table_right:1};
const $=id=>document.getElementById(id);
const esc=value=>String(value??"--").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
const number=value=>typeof value==="number"&&Number.isFinite(value);
const fmt=(value,digits=3)=>number(value)?value.toFixed(digits):"--";
const signed=(value,digits=3)=>number(value)?(value>=0?"+":"")+fmt(value,digits):"--";
const pair=value=>Array.isArray(value)?value.map(x=>signed(x)).join(" / "):"--";
const clock=value=>number(value)?new Date(value*1000).toLocaleTimeString("zh-CN",{hour12:false}):"--";
const duration=value=>{value=number(value)?value:0;return `${String(Math.floor(value/60)).padStart(2,"0")}:${(value%60).toFixed(1).padStart(4,"0")}`;};
let snapshot=null,connected=false,mode="live",frozen=false,playing=false;
let recordingId="",playbackTime=0,playbackDuration=0,tickPromise=null,dirtySeek=false;
let lastTick=performance.now(),lastLiveFetch=0,liveRecording={};
function topic(key){return snapshot?.topics?.[key]||{};}
function payload(key){return topic(key).payload||{};}
function fresh(key,limit=.25){return(mode==="replay"||connected)&&number(topic(key).age_s)&&topic(key).age_s<=limit;}
function planner(){return payload("planner");}
function latestDecision(){return planner().rl_decision||snapshot?.decisions?.at(-1)||null;}
function robotData(name){
  const state=payload("state/"+name),command=payload("command/"+name),torso=payload("torso/"+name);
  const stateValid=fresh("state/"+name)&&state.valid===true;
  const torsoValid=fresh("torso/"+name)&&Array.isArray(torso.position_xyz);
  const position=stateValid&&Array.isArray(state.base_position_xyz)?state.base_position_xyz:torsoValid?torso.position_xyz:null;
  const wire=command.command||{},desired=wire.desired_base_position;
  const commandValid=fresh("command/"+name)&&command.valid===true;
  const publishedTarget=commandValid&&Array.isArray(desired)?desired[1]:null;
  const plan=planner().robots?.[name]||{};
  const stage=planner().staging||{};
  const stageTarget=stage.targets?.[name];
  const sessionMatch=stateValid&&state.last_planner_session_id===planner().session_id;
  const sequenceAck=sessionMatch&&number(state.last_applied_sequence)&&number(stage.sequence?.[name])&&state.last_applied_sequence>=stage.sequence[name];
  const targetMatch=number(stageTarget)&&number(state.target_base_y)&&Math.abs(stageTarget-state.target_base_y)<=1e-4;
  return{state,command,torso,wire,plan,stage,stateValid,torsoValid,position,
    source:stateValid?"Controller base":torsoValid?"torso 定位":"无新鲜定位",
    commandValid,publishedTarget,stageTarget,sessionMatch,sequenceAck,targetMatch};
}
function error(message){$("error").hidden=!message;$("error").textContent=message||"";}
async function api(path,method="GET"){
  const response=await fetch(path,{method,cache:"no-store",headers:method==="POST"?{"Content-Type":"application/json"}:{}});
  const data=await response.json();if(!response.ok)throw new Error(data.error||`HTTP ${response.status}`);return data;
}
function count(state,newName,oldName){return state?.[newName]??state?.[oldName]??0;}
function renderRecording(){
  const state=liveRecording||{};
  $("record-dot").className=state.active?"recording":"";
  $("record-state").textContent=state.error?"录制写入异常":state.active?"正在录制原始消息":"录制已停止";
  $("record-count").textContent=`${state.id||"未开始"} · 消息 ${count(state,"messages","messages_written")} · 帧 ${count(state,"frames","frames_written")} · 队列丢弃 ${state.dropped||0}${state.error?" · "+state.error:""}`;
  $("record-start").disabled=!!state.active;$("record-stop").disabled=!state.active;
}
function formatFeatures(namesList,values){
  if(!Array.isArray(namesList)||!Array.isArray(values))return"暂无决策";
  return namesList.map((name,index)=>`${String(index).padStart(2,"0")}  ${name.padEnd(43," ")} ${fmt(values[index],6)}`).join("\n");
}
function renderDecision(){
  const record=latestDecision(),decision=record?.decision||{},current=fresh("planner",.8);
  const sameSession=!!record&&record.session_id===planner().session_id;
  const applied=sameSession?(decision.applied_pair_m||decision.guard_projected_m):null;
  [["table_left","left"],["table_right","right"]].forEach(([name,side])=>{
    const index=indexes[name],robot=robotData(name);
    $("applied-y-"+side).textContent=signed(applied?.[index]);
    $("position-y-"+side).textContent=signed(robot.position?.[1]);
    $("fixed-y-"+side).textContent=signed(planner().fixed_commands?.[name]?.desired_base_position?.[1]);
  });
  const gap=Array.isArray(applied)&&number(applied[0])&&number(applied[1])?applied[0]-applied[1]:null;
  $("guard-state").textContent=!sameSession?"等待决策":decision.valid!==true?"拒绝 / fallback":decision.guard_intervened?"已安全修正":"直接采用";
  $("guard-state").className=decision.valid===false?"bad-text":decision.guard_intervened?"ack-wait":"ack-good";
  const interval=record?.feature_sources?.interval;
  $("decision-context").textContent=sameSession
    ?`双机间隔 ${fmt(gap)} m · 决策延迟 ${fmt(decision.latency_ms,3)} ms · 来球间隔 ${fmt(interval?.clipped_s,3)} s`
    :"等待本会话 RL Planner command";
  const source=name=>{const r=robotData(name);return r.stateValid?"Controller":r.torsoValid?"torso":"无新鲜定位";};
  $("position-context").textContent=`198：${source("table_left")} · 66：${source("table_right")}${current?"":" · Planner 数据已过期"}`;
  const fixed=planner().fixed_commands||{};
  $("fixed-context").textContent=names.some(name=>fixed[name])
    ?`198：${fixed.table_left?.role||"--"} · 66：${fixed.table_right?.role||"--"}`
    :"等待 Rule-based command";
  $("actor-features").textContent=formatFeatures(record?.actor_names,record?.actor_observation);
  $("audit-features").textContent=formatFeatures(record?.audit_names,record?.audit_observation);
}
const phases=["HOME_HOLD","HIT","POST_DELAY","OUTWARD","OUTWARD_HOLD","RETURN"];
const phaseLabels={HOME_HOLD:"回位等待",HIT:"击球",POST_DELAY:"击球冷却",OUTWARD:"让位",OUTWARD_HOLD:"让位等待",RETURN:"回位"};
function ackDescription(robot){
  const stage=robot.stage;
  if(!stage.shot_id)return{className:"ack-wait",label:"未开始",detail:"当前没有 staging pair"};
  if(stage.failure)return{className:"ack-bad",label:"失败",detail:stage.failure};
  if(stage.complete&&robot.plan.ack===true)return{className:"ack-good",label:"完整 ACK",detail:`sequence ${stage.sequence?.[robot.state.robot]??"--"}`};
  const listed=(stage.acknowledged||[]).includes(robot.state.robot||robot.command.robot);
  return{className:listed?"ack-good":"ack-wait",label:listed?"本侧已 ACK":"等待 ACK",detail:`sequence ${stage.sequence?.[robot.state.robot||robot.command.robot]??"--"}`};
}
function renderRobot(name,id){
  const r=robotData(name),phase=r.stateValid?r.state.phase:null,ack=ackDescription(r);
  const transport=r.stateValid?r.state.transport_error:null;
  const statusClass=!r.stateValid||r.state.emergency_stop||transport?"bad":r.state.ready?"good":"warn";
  const statusText=!r.stateValid?"状态过期 / 缺失":r.state.emergency_stop?"EMERGENCY STOP":transport?"TRANSPORT ERROR":r.state.ready?"状态在线":"未 ready";
  $(id).innerHTML=`<div class="robot-title"><h3>${labels[name]}</h3><span class="badge ${statusClass}">${statusText}</span></div>
    <div class="robot-measures">
      <div><span>实时位置 Y</span><strong class="${name==="table_left"?"left-text":"right-text"}">${signed(r.position?.[1])}</strong><small>${esc(r.source)}</small></div>
      <div><span>Controller target Y</span><strong>${signed(r.stateValid?r.state.target_base_y:null)}</strong><small>last applied #${esc(r.state.last_applied_sequence)}</small></div>
      <div><span>STAGE ACK</span><strong class="${ack.className}">${ack.label}</strong><small>${esc(ack.detail)}</small></div>
    </div>
    <div class="phase-strip" aria-label="当前控制阶段">${phases.map(item=>`<span class="${phase===item?"is-current":""}" title="${item}">${phaseLabels[item]}</span>`).join("")}</div>
    <div class="robot-detail">
      当前 phase：<b>${esc(phase||"--")}</b>　·　Planner role：<b>${esc(r.plan.role||r.wire.role||"--")}</b>　·　planned active：<b>${esc(r.plan.planned_active)}</b><br>
      command target：${signed(r.publishedTarget)}　·　stage target：${signed(r.stageTarget)}　·　command published：${esc(r.plan.command_published)}<br>
      session match：${r.sessionMatch?"YES":"NO"}　·　sequence ACK：${r.sequenceAck?"YES":"NO"}　·　target match：${r.targetMatch?"YES":"NO"}<br>
      Controller session：<span class="mono">${esc(r.state.last_planner_session_id)}</span><br>
      transport_error：<b class="${transport?"bad-text":""}">${esc(transport||"none")}</b>　·　ready：${esc(r.state.ready)}　·　valid：${esc(r.state.valid)}　·　e-stop：${esc(r.state.emergency_stop)}
    </div>`;
}
function canvasContext(id){
  const canvas=$(id),box=canvas.getBoundingClientRect(),scale=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(box.width*scale)||canvas.height!==Math.round(box.height*scale)){canvas.width=Math.round(box.width*scale);canvas.height=Math.round(box.height*scale);}
  const ctx=canvas.getContext("2d");ctx.setTransform(scale,0,0,scale,0,0);ctx.clearRect(0,0,box.width,box.height);ctx.font="11px 'DejaVu Sans Mono',monospace";return{ctx,w:box.width,h:box.height};
}
function drawScene(){
  const {ctx,w,h}=canvasContext("scene"),robots=names.map(robotData),positions=robots.map(r=>r.position).filter(p=>Array.isArray(p)&&p.every(number));
  const ys=positions.map(p=>p[1]),xmin=Math.min(-.9,...positions.map(p=>p[0]-.3)),xmax=Math.max(3.25,...positions.map(p=>p[0]+.3));
  const ymin=Math.min(-1.1,...ys.map(y=>y-.25)),ymax=Math.max(1.1,...ys.map(y=>y+.25));
  const px=y=>38+(ymax-y)/(ymax-ymin)*(w-76),py=x=>h-32-(x-xmin)/(xmax-xmin)*(h-50);
  ctx.strokeStyle="#dce3d9";ctx.lineWidth=1;ctx.fillStyle="#7a8880";
  for(let y=-1;y<=1.001;y+=.5){ctx.beginPath();ctx.moveTo(px(y),18);ctx.lineTo(px(y),h-30);ctx.stroke();ctx.fillText(y.toFixed(1),px(y)-10,h-13);}
  const tableNear=.45,tableFar=3.19,net=1.82;
  ctx.fillStyle="#d9e8dc";ctx.fillRect(px(.76),py(tableFar),px(-.76)-px(.76),py(tableNear)-py(tableFar));
  ctx.strokeStyle="#a0b8a6";ctx.strokeRect(px(.76),py(tableFar),px(-.76)-px(.76),py(tableNear)-py(tableFar));
  ctx.beginPath();ctx.moveTo(px(.76),py(net));ctx.lineTo(px(-.76),py(net));ctx.stroke();ctx.fillStyle="#69846f";ctx.fillText("NET",px(.76)+7,py(net)-6);
  ctx.fillStyle="#697b73";ctx.fillText("+Y ←",8,h-13);ctx.fillText("→ -Y",w-47,h-13);ctx.fillText("+X ↑",8,17);
  const decision=latestDecision()?.decision||{},applied=decision.applied_pair_m||decision.guard_projected_m;
  robots.forEach((r,index)=>{
    const name=names[index],p=r.position;if(!Array.isArray(p)||!p.every(number))return;
    const x=px(p[1]),y=py(p[0]);ctx.strokeStyle=colors[name];ctx.fillStyle=colors[name];ctx.lineWidth=2;
    if(number(r.publishedTarget)){const gx=px(r.publishedTarget);ctx.setLineDash([5,4]);ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(gx,y);ctx.stroke();ctx.setLineDash([]);ctx.fillRect(gx-5,y-5,10,10);}
    const target=applied?.[indexes[name]];
    if(number(target)&&fresh("planner",.8)){ctx.setLineDash([2,3]);ctx.strokeRect(px(target)-10,y-10,20,20);ctx.setLineDash([]);ctx.fillText("applied "+fmt(target,2),Math.min(w-115,Math.max(4,px(target)-32)),y+26);}
    ctx.beginPath();ctx.arc(x,y,9,0,Math.PI*2);if(r.stateValid)ctx.fill();else ctx.stroke();
    ctx.fillStyle=colors[name];ctx.fillText(shortLabels[name]+"  "+fmt(p[1],2),Math.min(w-115,Math.max(4,x-35)),y-18);
  });
  const ball=payload("ball");if(fresh("ball",.3)&&ball.valid===true&&Array.isArray(ball.position)&&ball.position.every(number)){ctx.fillStyle="#dd692c";ctx.beginPath();ctx.arc(px(ball.position[1]),py(ball.position[0]),4,0,Math.PI*2);ctx.fill();}
  if(!positions.length){ctx.fillStyle="#77877e";ctx.textAlign="center";ctx.fillText("等待新鲜有效的机器人定位",w/2,h-55);ctx.textAlign="left";}
}
function drawTrajectory(){
  const {ctx,w,h}=canvasContext("trajectory"),rows=snapshot?.timeline||[],end=rows.at(-1)?.t??0,start=end-30;
  const all=rows.flatMap(row=>names.flatMap(name=>[row.robots?.[name]?.y,row.robots?.[name]?.target_y])).filter(number);
  const extent=Math.max(1.0,...all.map(Math.abs))+.1,px=t=>45+(t-start)/30*(w-61),py=y=>15+(extent-y)/(2*extent)*(h-45);
  ctx.strokeStyle="#e3e7de";ctx.fillStyle="#748178";ctx.lineWidth=1;
  for(let y=-1;y<=1.001;y+=.5){ctx.beginPath();ctx.moveTo(45,py(y));ctx.lineTo(w-16,py(y));ctx.stroke();ctx.fillText(y.toFixed(1),7,py(y)+4);}
  for(let offset=0;offset<=30;offset+=5)ctx.fillText(`${offset-30}s`,px(start+offset)-10,h-8);
  names.forEach(name=>["y","target_y"].forEach(field=>{ctx.strokeStyle=colors[name];ctx.lineWidth=field==="y"?2:1.5;ctx.setLineDash(field==="y"?[]:[5,4]);ctx.beginPath();let previous=null;
    rows.forEach(row=>{const value=row.robots?.[name]?.[field];if(!number(value)||row.t<start){previous=null;return;}if(previous!==null&&row.t-previous<=.35)ctx.lineTo(px(row.t),py(value));else ctx.moveTo(px(row.t),py(value));previous=row.t;});ctx.stroke();ctx.setLineDash([]);}));
  if(!rows.length){ctx.fillStyle="#748178";ctx.fillText("暂无轨迹，不补造历史位置",55,h/2);}
}
function renderTables(){
  $("quality").innerHTML=Object.entries(snapshot?.topics||{}).map(([name,item])=>{const limit=name==="planner"?.8:name==="ball"?.3:.25;return`<tr><td>${esc(name)}</td><td class="mono">${fmt(item.hz,1)}</td><td class="mono ${fresh(name,limit)?"":"bad-text"}">${fmt(item.age_s*1000,0)} ms</td><td class="mono">${item.sequence_gaps||0} / ${item.sequence_resets||0}</td></tr>`;}).join("");
  const record=snapshot?.recording||{},journal=planner().journal||{};
  $("quality-summary").textContent=`${mode==="replay"?"记录帧":"实时接收"} · Monitor 丢弃 ${record.dropped||0} · Planner journal 丢弃 ${journal.dropped||0} · JSON 解析错误 ${snapshot?.ros?.parse_errors||0}${record.error?" · 写入错误 "+record.error:""}`;
  $("events").innerHTML=(snapshot?.transitions||[]).slice().reverse().map(event=>`<div class="event-row"><span class="event-time">${clock(event.time)}</span><span>${esc(event.topic)}</span><span class="event-detail"><strong>${esc(event.label)}</strong> ${event.shot_id?esc(event.shot_id):""} ${number(event.y)?"Y="+fmt(event.y):""} ${esc(event.status||"")} ${esc(event.detail||"")}</span></div>`).join("")||'<p class="empty">等待状态变化</p>';
  $("decisions").innerHTML=(snapshot?.decisions||[]).slice().reverse().map(record=>{const d=record.decision||{},applied=d.applied_pair_m||d.guard_projected_m||[],gap=number(applied[0])&&number(applied[1])?applied[0]-applied[1]:null;return`<tr><td>#${esc(record.sequence)}<small>${clock(record.observed_stamp)}</small></td><td>${esc(record.shot_id)}</td><td class="mono">${pair(d.actor_raw)}</td><td class="mono">${pair(d.actor_physical_m)}</td><td class="mono">${pair(applied)}</td><td class="mono">${fmt(gap)}</td><td class="mono">${fmt(d.latency_ms,3)} ms</td><td>${d.valid===true?(d.guard_intervened?"guard 投影":"直通"):"拒绝 / fallback"}</td></tr>`;}).join("")||'<tr><td colspan="8" class="empty">等待 Actor 决策</td></tr>';
  renderRaw();
}
function renderRaw(){const name=$("raw-topic").value;$("raw").textContent=JSON.stringify(name==="all"?snapshot:topic(name),null,2);}
function render(){
  const p=planner(),ball=payload("ball"),stage=p.staging||{},ackCount=(stage.acknowledged||[]).length;
  $("connection").textContent=connected?"观测服务在线":"服务未连接";$("connection").className="badge "+(connected?"good":"bad");
  $("view-mode").textContent=mode==="replay"?"历史回放 / 非实时":frozen?"画面暂停 / 非实时":"实时视图";$("view-mode").className="badge "+(mode==="replay"||frozen?"warn":"good");
  $("planner-mode").textContent=p.mode||"等待 Planner";$("planner-mode").className="badge "+(p.mode==="active"?"good":p.mode?"warn":"bad");
  $("session").textContent=p.session_id||"未收到 Planner 会话";$("shot").textContent=stage.shot_id||ball.shot_id||"无有效来球";
  $("hitter").textContent=shortLabels[p.cycle_hitter]||"未知 / 无调度拍次";$("reason").textContent=p.reason||"等待输入";
  $("pair-ack").textContent=stage.complete?"2 / 2 完整 ACK":stage.failure?"失败："+stage.failure:stage.shot_id?`${ackCount} / 2 等待`:"未开始";
  $("pair-ack").className=stage.complete?"ack-good":stage.failure?"ack-bad":"ack-wait";
  $("scene-context").textContent=`${mode==="replay"||frozen?"记录帧 / ":""}${clock(snapshot?.wall_time_s)} · origin 坐标 / 米`;
  $("play").disabled=mode!=="replay";$("play").textContent=playing?"暂停回放":"播放";$("freeze").disabled=mode!=="live";$("freeze").textContent=frozen?"恢复画面":"暂停画面";
  $("scrub").disabled=mode!=="replay";$("scrub").max=playbackDuration;$("scrub").value=playbackTime;
  $("play-time").textContent=duration(playbackTime)+" / "+duration(playbackDuration);
  const frame=snapshot?.playback?.frame??snapshot?.playback?.frame_index;
  $("frame-label").textContent=snapshot?.playback?`帧 ${frame+1} / ${snapshot.playback.frame_count}`:`已接收消息 ${snapshot?.event_id||0}`;
  renderRecording();renderDecision();renderRobot("table_left","robot-left");renderRobot("table_right","robot-right");drawScene();drawTrajectory();renderTables();
}
async function refreshRecordings(){
  const result=await api("/api/recordings"),selected=$("recordings").value;
  $("recordings").innerHTML='<option value="">选择一段测试记录</option>'+(result.recordings||[]).map(item=>`<option value="${esc(item.id)}" ${item.active?"disabled":""}>${esc(item.id)} · ${duration(item.duration_s||0)}${item.active?" / 录制中":""}${item.dropped?" / 有丢弃":""}</option>`).join("");
  $("recordings").value=selected;updateDownload();
}
function updateDownload(){const id=$("recordings").value;$("download").className="button"+(id?"":" disabled");$("download").href=id?`/api/recordings/${encodeURIComponent(id)}/messages`:"#";}
async function action(fn){try{await fn();error("");}catch(e){error(e.message);}}
$("record-start").onclick=()=>action(async()=>{liveRecording=await api("/api/record/start","POST");renderRecording();await refreshRecordings();});
$("record-stop").onclick=()=>action(async()=>{liveRecording=await api("/api/record/stop","POST");renderRecording();await refreshRecordings();});
$("refresh-recordings").onclick=()=>action(refreshRecordings);$("recordings").onchange=updateDownload;
$("load-recording").onclick=()=>action(async()=>{const id=$("recordings").value;if(!id)throw new Error("请先选择已保存记录");const frame=await api(`/api/recordings/${encodeURIComponent(id)}/frame?time=0`);mode="replay";recordingId=id;playing=false;frozen=false;playbackTime=0;playbackDuration=frame.playback.duration_s;snapshot=frame;render();});
$("live").onclick=()=>{mode="live";frozen=false;playing=false;playbackTime=playbackDuration=0;lastLiveFetch=0;};
$("freeze").onclick=()=>{frozen=!frozen;render();};$("play").onclick=()=>{if(playbackTime>=playbackDuration)playbackTime=0;playing=!playing;dirtySeek=true;render();};
$("scrub").oninput=()=>{playbackTime=Number($("scrub").value);dirtySeek=true;return tick();};$("raw-topic").onchange=renderRaw;
window.addEventListener("resize",()=>{drawScene();drawTrajectory();});
function tick(){if(tickPromise)return tickPromise.then(()=>dirtySeek?tick():undefined);tickPromise=updateView().finally(()=>{tickPromise=null;});return tickPromise;}
async function updateView(){
  const now=performance.now(),delta=Math.min((now-lastTick)/1000,.5);lastTick=now;
  try{
    if(mode==="live"||now-lastLiveFetch>1000){const data=await api("/api/snapshot");connected=true;lastLiveFetch=now;liveRecording=data.recording||{};renderRecording();if(mode==="live"&&!frozen){snapshot=data;render();}}
    if(mode==="replay"&&(playing||dirtySeek)){if(playing)playbackTime=Math.min(playbackDuration,playbackTime+delta*Number($("speed").value));if(playbackTime>=playbackDuration)playing=false;dirtySeek=false;const id=recordingId,requested=playbackTime,frame=await api(`/api/recordings/${encodeURIComponent(id)}/frame?time=${requested}`);if(mode==="replay"&&id===recordingId&&requested===playbackTime){snapshot=frame;render();}}
  }catch(e){connected=false;error("观测服务连接或回放异常："+e.message);if(snapshot)render();}
}
action(refreshRecordings);setInterval(tick,100);tick();
