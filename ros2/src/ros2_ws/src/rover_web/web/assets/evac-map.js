window.drawEvacuationMap = function(canvas, model, pose, path = [], selected = null) {
  const c = canvas.getContext('2d');
  const minimal = Boolean(model.chatMinimal);
  canvas.width = 760; canvas.height = 820;
  const size = 136, left = 40, top = 70;
  const center = (p) => [left + (4 - p[0] + .5) * size, top + (4 - p[1] + .5) * size];
  c.fillStyle = '#f3f7fc'; c.fillRect(0, 0, 760, 820);
  c.fillStyle = '#123047'; c.font = 'bold 24px sans-serif'; if(!minimal)c.fillText('Эвакуация · 5 × 5 · клетка 0,8 м', 40, 40);
  const colors = {road:'#fff',rubble:'#e8d598',yellow:'#ffe49b',blue:'#b1d6f6',river:'#9cddeb',bridge:'#ffc5b9',beyond:'#ffc5b9'};
  for (const cell of model.cells) {
    const [cx, cy] = center([cell.x,cell.y]), x = cx-size/2, y=cy-size/2;
    c.fillStyle=colors[cell.kind] || '#fff';c.fillRect(x,y,size,size);c.strokeStyle='#b1c4d4';c.lineWidth=1;c.strokeRect(x,y,size,size);
    if(!minimal && ['bridge','beyond','river'].includes(cell.kind)) {
      c.save();c.beginPath();c.rect(x,y,size,size);c.clip();c.strokeStyle='#d8494933';
      for(let t=-size;t<size*2;t+=18){c.beginPath();c.moveTo(x+t,y);c.lineTo(x+t+size,y+size);c.stroke();}c.restore();
    }
    if(minimal && cell.x === 1 && cell.y === 1) {c.fillStyle='#334c63';c.font='16px sans-serif';c.fillText('Парковка',x+12,y+26);}
    if(minimal && cell.kind !== 'road') {
      const names={rubble:'Обломки',yellow:'Дом',blue:'Дом',river:'Водоём',bridge:'Мост',beyond:''};
      c.fillStyle='#334c63';c.font='16px sans-serif';c.fillText(names[cell.kind]||'',x+12,y+26);
    }
    if(!minimal) {
    c.fillStyle='#536979';c.font='13px sans-serif';c.fillText(`(${cell.x}, ${cell.y})`,x+8,y+18);
    c.font='12px sans-serif';const words=cell.description.split(/\s+/);let line='',ly=y+36;
    for(const word of words){if(c.measureText(line+word).width>size-14){c.fillText(line,x+7,ly);line='';ly+=15;if(ly>y+70)break;}line+=word+' ';}if(ly<=y+70)c.fillText(line,x+7,ly);
    }
    if(selected && selected[0]===cell.x && selected[1]===cell.y){c.strokeStyle='#0458bd';c.lineWidth=4;c.strokeRect(x+3,y+3,size-6,size-6);}
  }
  if(path.length){c.strokeStyle='#7546da';c.lineWidth=6;c.beginPath();path.forEach((p,i)=>{const [x,y]=center(p);i?c.lineTo(x,y):c.moveTo(x,y);});c.stroke();}
  if(path.length && model.person) {
    const vectors=[[1,0],[0,1],[-1,0],[0,-1]];
    const [dx,dy]=vectors[model.person.facing];
    const target=model.person.cell;
    const adjacent=[target[0]+dx,target[1]+dy];
    const end=path[path.length-1];
    if(Math.abs(end[0]-adjacent[0])<.001 && Math.abs(end[1]-adjacent[1])<.001) {
      const [sx,sy]=center(adjacent);
      const [ex,ey]=center([target[0]+dx*.25,target[1]+dy*.25]);
      c.save();c.strokeStyle='#7546da';c.fillStyle='#7546da';c.lineWidth=5;
      c.setLineDash([10,7]);c.beginPath();c.moveTo(sx,sy);c.lineTo(ex,ey);c.stroke();
      c.setLineDash([]);
      const angle=Math.atan2(ey-sy,ex-sx);
      c.beginPath();c.moveTo(ex,ey);
      c.lineTo(ex-15*Math.cos(angle-.45),ey-15*Math.sin(angle-.45));
      c.lineTo(ex-15*Math.cos(angle+.45),ey-15*Math.sin(angle+.45));c.closePath();c.fill();
      c.beginPath();c.arc(ex,ey,8,0,Math.PI*2);c.stroke();
      c.restore();
    }
  }
  const marks={accident:['#1685ee','Авария'],tanker:['#ed8b25','Бензовоз'],tree:['#27a456','Дерево'],bus_stop:['#9364d5','Остановка'],rubble:['#b49456','Обломки']};
  for(const [kind,p] of Object.entries(model.markers)){const [x,y]=center(p);c.fillStyle=marks[kind][0];c.beginPath();c.arc(x,y+24,19,0,2*Math.PI);c.fill();c.fillStyle='#19384a';c.font='12px sans-serif';if(!minimal)c.fillText(marks[kind][1],x-29,y+55);}
  const arrow=(p,d,color)=>{const [x,y]=center(p);const vectors=[[-1,0],[0,-1],[1,0],[0,1]], [dx,dy]=vectors[d];c.strokeStyle=color;c.lineWidth=5;c.beginPath();c.moveTo(x,y);c.lineTo(x+dx*38,y+dy*38);c.stroke();c.beginPath();c.moveTo(x+dx*38,y+dy*38);c.lineTo(x+dx*23-dy*9,y+dy*23+dx*9);c.moveTo(x+dx*38,y+dy*38);c.lineTo(x+dx*23+dy*9,y+dy*23-dx*9);c.stroke();};
  if(model.person){const [x,y]=center(model.person.cell);c.fillStyle='#e64450';c.fillRect(x-17,y-17,34,34);arrow(model.person.cell,model.person.facing,'#b51425');}
  if(pose){const [x,y]=center(pose.cell);c.fillStyle='#132f49';c.beginPath();c.arc(x,y,15,0,Math.PI*2);c.fill();arrow(pose.cell,pose.heading,'#132f49');}
  if(minimal)return;
  c.fillStyle='#334c63';c.font='15px sans-serif';c.fillText('X ←     Y ↑     Старт (0,0) справа внизу. Робот изначально смотрит ↑',40,780);
  c.font='13px sans-serif';c.fillText('Красный — человек; красная стрелка — куда смотрит QR. Фиолетовый — маршрут.',40,804);
};
