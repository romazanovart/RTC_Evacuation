const $=s=>document.querySelector(s);
let model=null,pose={cell:[0,0],heading:1},plan=null,tool='accident',editing=true,dirty=false,running=false,epoch=null,lastEvent=0,visible={},personVisible=false;
let editVersion=0,savePending=false,posePending=false,saveTask=null;
function changed(withPose=false){
  editVersion++;dirty=true;savePending=true;posePending=posePending||withPose;
  plan=null;if($('#start'))$('#start').disabled=true;$('#status').textContent='Сохраняю…';draw();
  if(!saveTask)saveTask=saveEdits().finally(()=>{saveTask=null;});
  return saveTask;
}
async function saveEdits(){
  while(savePending){
    if(running){$('#status').textContent='Не сохранено: выполняется движение';return;}
    savePending=false;
    const version=editVersion,withPose=posePending;
    posePending=false;
    const payload={map:structuredClone(model)};
    if(withPose)payload.pose={cell:[...pose.cell],heading:1};
    try{
      const s=await api('/api/evacuation/config',payload);
      if(version===editVersion){model=s.map;pose=s.pose;dirty=false;visible={};personVisible=false;$('#status').textContent='Сохранено';draw();}
    }catch(e){
      posePending=posePending||withPose;
      if(version===editVersion){$('#status').textContent='Не сохранено: '+e.message;return;}
    }
  }
}
const names={accident:'Авария',tanker:'Бензовоз',bus_stop:'Остановка',tree:'Дерево',rubble:'Обломки',person:'Человек / QR',erase:'Стереть'};
async function api(path,data){const r=await fetch(path,{method:data===undefined?'GET':'POST',headers:{'Content-Type':'application/json'},body:data===undefined?undefined:JSON.stringify(data),signal:AbortSignal.timeout(30000)});const b=await r.json();if(!r.ok)throw Error(b.error||b.message||r.status);return b;}
function message(text,image){if(!$('#chat'))return;const a=document.createElement('article');a.className='message';a.textContent=text;if(image&&image.startsWith('data:image/jpeg;base64,')){const i=document.createElement('img');i.src=image;i.alt='QR, считанный камерой';a.append(i);}$('#chat').append(a);$('#chat').scrollTop=$('#chat').scrollHeight;}
function draw(){if(!model)return;const m=structuredClone(model);if(!editing){m.markers=visible;if(!personVisible)m.person=null;}drawEvacuationMap($('#map'),m,pose,plan?.path||[]);$('#editing').hidden=!editing;}
function bind(id,fn){if(!$(id))return;$(id).onclick=async()=>{try{await fn();}catch(e){$('#status').textContent=e.message;message(e.message);}};}
for(const [key,name] of Object.entries(names)){const b=document.createElement('button');b.textContent=name;b.onclick=()=>{tool=key;$('#tools').querySelectorAll('button').forEach(x=>x.classList.remove('active'));b.classList.add('active');};$('#tools').append(b);}
$('#map').onclick=e=>{if(!editing||running||!model)return;const r=e.target.getBoundingClientRect();const p=[4-Math.floor(((e.clientX-r.left)*760/r.width-40)/136),4-Math.floor(((e.clientY-r.top)*820/r.height-70)/136)];if(p.some(v=>v<0||v>4))return;const c=model.cells.find(c=>c.x===p[0]&&c.y===p[1]);const objectCell=['road','bridge','beyond'].includes(c.kind);if(tool==='robot'&&c.kind!=='road'){$('#status').textContent='Робот может стоять только на дороге';return;}if(tool!=='robot'&&!objectCell){$('#status').textContent='Объект можно поставить на дороге, мосту или за мостом';return;}if(tool==='robot')pose={cell:p,heading:1};else if(tool==='person')model.person={cell:p,facing:Number($('#facing').value)};else {for(const[k,v]of Object.entries(model.markers))if(v.join()===p.join())delete model.markers[k];if(tool==='erase'){if(model.person?.cell.join()===p.join())model.person=null;}else model.markers[tool]=p;}changed(tool==='robot');};
$('#facing').onchange=()=>{if(running)return;if(model?.person){model.person.facing=Number($('#facing').value);changed();}};
bind('#editor',()=>{editing=true;draw();});bind('#mission-view',()=>{editing=false;draw();});
bind('#save',async()=>{if(running)throw Error('Сначала останови движение');await changed(true);});
bind('#preview-route',async()=>{if(running)throw Error('Сначала останови движение');if(saveTask)await saveTask;if(dirty)throw Error('Сначала сохрани карту');plan=await api('/api/route/preview',{});$('#status').textContent='Маршрут показан. Движение не запущено.';draw();});
bind('#prepare',async()=>{if(dirty)throw Error('Сначала сохрани карту');$('#prepare').disabled=true;try{const text=$('#task').value;message('Задание: '+text);const r=await api('/api/mission/prepare',{text});plan=r.plan;if(r.keywords.length)message('Ключевые слова: '+r.keywords.join(' · '));message(r.reply);message('Маршрут: '+plan.path.map(p=>'('+p.join(', ')+')').join(' → '));editing=false;draw();await api('/api/evacuation/start',{plan_id:plan.id});$('#start').disabled=true;}finally{$('#prepare').disabled=false;}});
bind('#start',async()=>{if(!plan||dirty)throw Error('Построй маршрут');$('#start').disabled=true;await api('/api/evacuation/start',{plan_id:plan.id});message(['Начинаю движение по проверенному маршруту.','Принято, выдвигаюсь к первой точке.','Поехали. Сначала достигну указанного ориентира.'][Math.floor(Math.random()*3)]);});
bind('#stop',async()=>{await api('/api/evacuation/stop',{});$('#status').textContent='Остановка отправлена.';});
async function refresh(){const version=editVersion,wasSaving=dirty||Boolean(saveTask);try{const s=await api('/api/evacuation/status');running=s.running;if(epoch!==s.epoch){epoch=s.epoch;lastEvent=0;visible={};personVisible=false;}if(!dirty&&!wasSaving&&version===editVersion){model=s.map;pose=s.pose;}for(const e of s.events||[]){if(e.id<=lastEvent)continue;lastEvent=e.id;if(e.map){visible=e.map.markers||{};personVisible=Boolean(e.map.person);}if(e.path)plan={...plan,path:e.path};message(e.text+(e.path?'\n'+e.path.map(p=>'('+p.join(', ')+')').join(' → '):''),e.imageData);}if(!dirty)$('#status').textContent=running?'Выполняется маршрут':'Робот остановлен';draw();}catch(e){if(!dirty)$('#status').textContent='Связь: '+e.message;}finally{setTimeout(refresh,1500);}}
refresh();

$('#export-map').onclick=async()=>{
  const button=$('#export-map');button.disabled=true;
  $('#map-code-panel').hidden=false;$('#map-code-status').textContent='Создаю…';$('#map-code').value='';
  try{if(saveTask)await saveTask;if(dirty)throw Error('Сначала сохрани карту');const result=await api('/api/map-code');$('#map-code').value=result.code;$('#map-code-status').textContent='Код готов';}
  catch(e){$('#map-code-status').textContent=e.message;}
  finally{button.disabled=false;}
};
$('#copy-map-code').onclick=async()=>{
  const input=$('#map-code');if(!input.value)return;
  try{if(navigator.clipboard&&window.isSecureContext)await navigator.clipboard.writeText(input.value);else{input.focus();input.select();if(!document.execCommand('copy'))throw Error();}$('#map-code-status').textContent='Скопировано';}
  catch(e){input.focus();input.select();$('#map-code-status').textContent='Скопируй выделенный код вручную';}
};
