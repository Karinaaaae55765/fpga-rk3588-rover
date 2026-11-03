"use strict";
(() => {
  const $=id=>document.getElementById(id);
  const storageKey="rover.live.pairing.v1";
  function remember(value){try{if(value)localStorage.setItem(storageKey,value);else localStorage.removeItem(storageKey);return true;}catch{return false;}}
  function remembered(){try{return localStorage.getItem(storageKey)||"";}catch{return "";}}
  let token="",generation=0,state=null,lastState=0,telemetryDeadline=0,visionDeadline=0,imageDeadline=0,currentURL=null,actionBusy=false;
  const clientId=Date.now().toString(36)+Math.random().toString(36).slice(2);
  let ownedMission=null,motionBusy=false,heartbeatBusy=false;
  const messages=[],validNumber=value=>typeof value==="number"&&Number.isFinite(value);
  $("address").textContent=location.host;
  const reasons={
    show_one_object:"请将单个目标放入登记区", multiple_objects:"登记区域内有多个目标",
    object_not_fully_inside:"请将目标完整放入登记框", object_too_small:"目标太小，请适当靠近",
    image_not_clear:"图像不够清晰", hold_object_still:"请保持目标稳定",
    press_confirm:"目标已稳定，可以确认", frame_expired:"图像已过期",
    stream_restarted:"图像输入已重新启动", candidate_changed:"候选目标已变化，请重新确认",
    target_saved_no_motion:"已保存目标，不会启动小车", registered:"登记成功",
    already_registered:"该目标已登记", save_failed:"保存失败，请查看板端",
    session_changed:"服务已重新启动，请刷新状态", registration_unavailable:"登记服务不可用"
  };
  function log(text){messages.unshift(new Date().toLocaleTimeString("zh-CN",{hour12:false})+"  "+text);messages.splice(30);$("events").replaceChildren();for(const message of messages){const li=document.createElement("li");li.textContent=message;$("events").append(li);}}
  function clearFrame(){imageDeadline=0;$("video").hidden=true;$("video").removeAttribute("src");$("video-placeholder").hidden=false;if(currentURL){URL.revokeObjectURL(currentURL);currentURL=null;}}
  function clearLive(){clearFrame();state=null;lastState=telemetryDeadline=visionDeadline=0;render();}
  async function request(path,options={}){
    const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1800);
    try{
      const response=await fetch(path,{...options,cache:"no-store",signal:controller.signal,headers:{Authorization:"Bearer "+token,...options.headers}});
      // Consume the body while timeout is still active.
      const data=path==="/api/frame"&&response.ok?await response.blob():await response.json();
      if(response.status===401){const error=Error("配对码无效，请重新配对。");error.unauthorized=true;throw error;}
      if(!response.ok)throw Error(reasons[data.reason]||data.error||data.reason||"请求失败");
      return {response,data};
    }finally{clearTimeout(timer);}
  }
  function disconnect(message="页面已断开连接。"){
    if(ownedMission&&token)request("/api/motion/stop",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"}).catch(()=>{});
    ownedMission=null;
    generation++;token="";actionBusy=false;$("disconnect").disabled=true;clearLive();$("connection").textContent="未连接";$("pair-hint").textContent=message;
  }
  async function pair(value,automatic=false){
    disconnect(automatic?"正在使用本机保存的配对自动连接…":"正在验证配对码…");token=value.trim();$("pair-code").value="";
    if(!token)return;const mine=generation;$("pair-button").disabled=true;
    try{await request("/api/state");if(mine!==generation)return;$("pair-hint").textContent=remember(token)?"已在本浏览器记住配对；点击“断开并忘记配对”可清除。":"已连接；浏览器禁止保存配对，刷新后需重新输入。";$("disconnect").disabled=false;log("已连接板端服务。");pollState(mine);pollFrame(mine);}
    catch(error){if(mine===generation){
      if(error.unauthorized){remember("");disconnect(error.message);}
      else if(automatic){$("pair-hint").textContent="等待板端服务恢复，将自动重连。";$("disconnect").disabled=false;pollState(mine);pollFrame(mine);}
      else disconnect(error.message);
    }}
    finally{if(mine===generation||!token)$("pair-button").disabled=false;}
  }
  $("pair-form").addEventListener("submit",event=>{event.preventDefault();pair($("pair-code").value);});
  $("disconnect").addEventListener("click",()=>{remember("");disconnect("已断开并清除本浏览器保存的配对。");});
  async function pollState(mine){
    if(mine!==generation||!token)return;
    const started=performance.now();
    try{
      const {data}=await request("/api/state");if(mine!==generation)return;
      state=data;lastState=started;
      telemetryDeadline=data.telemetry?.connected?started+Math.max(0,500-(data.telemetry.age_ms??500)):0;
      visionDeadline=data.vision?.connected?started+Math.max(0,data.vision.expires_in_ms??0):0;
      if($("connection").textContent!=="设备服务已连接")$("pair-hint").textContent=remembered()===token?"已恢复连接，配对保存在本浏览器。":"已连接；当前配对未保存在浏览器中。";
      $("connection").textContent="设备服务已连接";render();
    }catch(error){if(mine!==generation)return;if(error.unauthorized){remember("");disconnect(error.message);return;}clearLive();$("connection").textContent="等待自动重连";$("pair-hint").textContent="设备服务暂不可用，将自动重连。";}
    setTimeout(()=>pollState(mine),250);
  }
  async function pollFrame(mine){
    if(mine!==generation||!token)return;
    const started=performance.now();
    try{
      const {response,data}=await request("/api/frame");if(mine!==generation)return;
      const ttl=Number(response.headers.get("X-Frame-TTL-Ms"));
      const deadline=started+Math.min(1000,Number.isFinite(ttl)?ttl:0);
      if(performance.now()>=deadline){clearFrame();return;}
      const url=URL.createObjectURL(data),image=new Image();
      image.src=url;
      try{await image.decode();}catch{URL.revokeObjectURL(url);throw Error("图像解码失败");}
      if(mine!==generation||performance.now()>=deadline){URL.revokeObjectURL(url);return;}
      if(currentURL)URL.revokeObjectURL(currentURL);
      currentURL=url;imageDeadline=deadline;$("video").src=url;$("video").hidden=false;$("video-placeholder").hidden=true;
    }catch{if(mine===generation)clearFrame();}
    finally{if(mine===generation&&token)setTimeout(()=>pollFrame(mine),180);}
  }
  function render(){
    const now=performance.now(),service=!!token&&!!state&&now-lastState<1200;
    const vision=service&&state.vision.connected&&now<visionDeadline;
    const telemetry=service&&state.telemetry.connected&&now<telemetryDeadline;
    $("video-status").textContent=vision?"有效帧":"无有效画面";
    $("npu-status").textContent=service?({initialized:"已初始化",starting:"启动中",error:"异常"}[state.vision.npu]||"未知"):"未知";
    $("uart-status").textContent=telemetry?"有效帧":"无有效遥测";
    $("cap-vision").textContent=vision?"已接收真实帧":"待验证";
    $("cap-telemetry").textContent=telemetry?"已接收有效帧":"待验证";
    const frame=vision?state.vision.frame:null;
    $("frame-id").textContent=frame?`FRAME ${frame.frame_id}`:"无有效帧";
    $("fps").textContent=vision&&validNumber(state.vision.average_processing_fps)?`${state.vision.average_processing_fps.toFixed(1)} FPS`:"—";
    $("inference").textContent=frame?`${frame.processing_ms} ms`:"—";
    $("frame-age").textContent=vision?`${state.vision.local_frame_age_ms} ms`:"—";
    $("vision-error").textContent=service?state.vision.error||"":"";
    $("detections").replaceChildren();
    const boxes=frame?.boxes||[];
    for(const box of boxes){const li=document.createElement("li");li.textContent=`${box.label} · ${(box.score*100).toFixed(1)}% · ${colorText(box.colors)}`;$("detections").append(li);}
    if(!boxes.length){const li=document.createElement("li");li.textContent=vision?"本帧未检出目标":"等待有效检测结果";$("detections").append(li);}
    const values=telemetry?state.telemetry.values:null;
    $("distance").textContent=validNumber(values?.distance_mm)?`${values.distance_mm} mm`:"—";
    for(const key of ["roll","pitch","yaw"])$(key).textContent=validNumber(values?.[key])?`${values[key].toFixed(2)}°`:"—";
    $("telemetry-info").textContent=service?`有效帧 ${state.telemetry.frames_ok} · 校验/格式错误 ${state.telemetry.crc_errors} · 重复/倒序 ${state.telemetry.ignored_sequence}`:"尚未收到有效串口帧";
    $("telemetry-error").textContent=service?state.telemetry.error||"": "";
    const reg=service?state.registration:null;
    $("registration-state").textContent=reg?({waiting:"等待目标",stabilizing:"观察中",ready:"可登记",registered:"已登记",saving:"保存中",error:"异常"}[reg.state]||reg.state):"未连接";
    $("registration-reason").textContent=reg?(reasons[reg.reason]||reg.reason):"等待板端登记服务。";
    $("candidate").textContent=vision&&reg?.candidate?`${reg.candidate.label} · ${(reg.candidate.score*100).toFixed(1)}%`:"—";
    $("stable").textContent=vision&&reg?`稳定帧 ${reg.stable_frames} / ${reg.required_frames}`:"稳定帧数 —";
    $("target-id").textContent=reg?.target_id||"—";
    $("confirm").disabled=actionBusy||!vision||!reg?.candidate_token;
    $("arm").disabled=actionBusy||!reg||!["registered","error"].includes(reg.state);
    renderMotion(service);
    draw();
  }
  async function action(operation){
    if(actionBusy||!state?.registration)return;
    const mine=generation,reg=state.registration;
    const body=operation==="confirm"?{candidate_token:reg.candidate_token}:{session_id:reg.session_id};
    if(operation==="confirm"&&$("confirm").disabled)return;
    actionBusy=true;render();
    try{const {data}=await request("/api/"+operation,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});if(mine!==generation)return;log(operation==="confirm"?`登记结果：${reasons[data.reason]||data.reason||"已保存"}`:"已开始新一轮登记。");
      // Invalidate action tokens until the next server response.
      state=null;render();
    }catch(error){if(mine===generation)log(error.message);}
    finally{if(mine===generation){actionBusy=false;render();}}
  }
  $("confirm").addEventListener("click",()=>action("confirm"));$("arm").addEventListener("click",()=>action("arm"));
  $("clear-events").addEventListener("click",()=>{messages.length=0;$("events").replaceChildren();});
  const colorNames={red:"红",orange:"橙",yellow:"黄",green:"绿",cyan:"青",blue:"蓝",purple:"紫",black:"黑",gray:"灰",white:"白",mixed:"混色",unknown:"未知"};
  function colorText(colors){return Object.entries(colors||{}).map(([region,value])=>`${({body:"主体",upper:"上身",lower:"下身"})[region]||region}：${colorNames[value.name]||"未知"}`).join(" / ")||"颜色未知";}
  const motionReasons={
    vehicle_verified:"车轮运动参数未实测验证",ground_projection_verified:"相机地面位置换算未标定",
    recognition_envelope_verified:"识别距离范围未验证",tof_extrinsics_verified:"测距安装关系未验证",
    "physical calibration not verified":"实车标定未完成",search_map_not_printed_map:"任务六仍为 6×4 m 地图，与当前实体打印地图不一致",
    initial_pose_required_or_stale:"请初始化位置并确认遥测有效",initial_pose_required:"请填写实车初始坐标和朝向",
    explicit_start_required:"等待主动启动任务",waiting_telemetry:"等待新鲜遥测",estimated_uncalibrated:"估计值（比例待核验）",
    telemetry_stale:"遥测已过期",vision_stale:"图像已过期",vision_not_ready:"等待有效识别帧",
    waiting_fresh_vision:"等待新目标识别结果",waiting_ARM_ack:"等待 FPGA 接受控制",
    ARM_ack_timeout_check_TX_wiring:"FPGA 未应答 ARM，请检查控制 TX 接线及固件状态",
    FPGA_not_ready_or_manual_control:"FPGA 未就绪或正在手动控制",stationary_telemetry_required:"需要新鲜遥测且车辆停稳",
    browser_lease_expired:"页面控制连接超时，已结束任务",user_stop:"用户停止",no_ui_mission:"当前没有 UI 任务",
    stop_requested:"已请求停止，等待遥测确认",calibration_or_map_required:"标定或地图条件未满足",
    stop_before_pose_reset:"请先停止任务，再初始化位置",initial_pose_set_estimate_only:"已设置起始坐标，仅为估计位置",
    initial_pose_outside_printed_map:"初始坐标超出实体地图范围",invalid_initial_pose:"请完整填写有效初始坐标和朝向",
    target_reached_and_stopped:"已接近目标并停稳",tracking_target:"正在接近/跟随目标",holding_distance:"保持距离",
    target_lost_or_changed:"目标丢失或改变，任务停止",multiple_matching_targets:"出现多个同类同色目标，任务停止",
    serial_unavailable:"串口未连接",mission_already_active:"已有任务运行",control_loop_stalled:"控制循环超时，任务停止",
    pose_invalid:"估计位置失效，任务停止",configuration_changed_restart_required:"配置已变更，需要重启服务",
    camera_restarted:"摄像头数据流重启，任务停止",target_range_unavailable:"目标距离无法可靠计算",
    target_approach_blocked:"目标附近通行条件不满足",target_not_visible:"没有锁定指定目标",
    mission_timeout:"任务超时",navigation_no_progress:"未取得有效接近进展",target_too_close:"目标过近",
    settling_at_target:"等待到达目标后停稳",mission_queued:"任务已提交，等待条件就绪"
  };
  const explain=value=>motionReasons[value]||value||"—";
  function renderMotion(service){
    const t=service?state.telemetry:null,m=t?.motion,p=t?.position;
    const fresh=!!m?.connected,mode=$("mission-type").value;
    const issues=m?.blockers?.[mode]||[];
    $("motion-state").textContent=m?({IDLE:"待命",PREPARING:"准备中",ARMING:"申请控制",RUNNING:"任务运行",STOPPING:"等待停车确认",STOPPED:"任务停止",COMPLETE:"任务完成",FAILED:"任务未完成"}[m.state]||m.state):"未连接";
    $("mission-reason").textContent=m?.active?explain(m.reason):(issues.length?[...new Set(issues.map(explain))].join("；"):m?.error||(!fresh?"等待有效任务六遥测":explain(m?.reason)));
    if(m?.stop_confirmed)$("mission-reason").textContent+="；遥测确认停稳";
    $("mission-start").disabled=motionBusy||!service||!fresh||!!m?.active||issues.length>0;
    $("mission-stop").disabled=!service||!m?.active;
    $("pose-set").disabled=motionBusy||!service||!fresh||!!m?.active;
    $("pose-status").textContent=p?.available?"里程计估计":explain(p?.reason||"initial_pose_required");
    $("pose-value").textContent=p?.available?`${p.pose.x.toFixed(2)}, ${p.pose.y.toFixed(2)}`:"—";
    $("wheel-status").textContent=fresh?`${m.task6_telemetry.left_rpm} / ${m.task6_telemetry.right_rpm} RPM`:"—";
    $("cap-color").textContent=service&&state.vision.connected?"已接入":"等待图像";
    $("cap-pose").textContent=p?.available?"估计位置":"待初始化/遥测";
    $("cap-motion").textContent=m?.active?"运行中":issues.length?"条件未满足":fresh?"可申请执行":"等待遥测";
    if(ownedMission&&m?.mission_id===ownedMission&&!m.active)ownedMission=null;
  }
  async function motionAction(path,body){
    if(motionBusy&&path!=="/api/motion/stop")return;
    motionBusy=true;render();
    try{
      const {data}=await request(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
      if(path==="/api/motion/start")ownedMission=data.mission_id;
      if(path==="/api/motion/stop")ownedMission=null;
      log(explain(data.reason||"已提交"));
    }catch(error){log(explain(error.message));}
    finally{motionBusy=false;render();}
  }
  $("mission-start").addEventListener("click",()=>{
    if($("mission-start").disabled)return;
    motionAction("/api/motion/start",{mode:$("mission-type").value,label:$("mission-label").value,color:$("mission-color").value||null,distance:Number($("mission-distance").value),client_id:clientId});
  });
  $("mission-stop").addEventListener("click",()=>motionAction("/api/motion/stop",{}));
  $("pose-set").addEventListener("click",()=>{
    const ids=["pose-x","pose-y","pose-heading"];
    if(ids.some(id=>!$(id).value.trim()||!Number.isFinite(Number($(id).value)))){log("请完整填写初始坐标和朝向。");return;}
    motionAction("/api/pose",{x:Number($("pose-x").value),y:Number($("pose-y").value),heading_deg:Number($("pose-heading").value)});
  });
  $("mission-type").addEventListener("change",()=>{
    const follow=$("mission-type").value==="follow-visible";
    $("mission-label").value=follow?"person":"cup";
    $("mission-distance").value=follow?"0.8":"0.5";
    render();
  });
  setInterval(async()=>{
    if(!ownedMission||!token||heartbeatBusy||document.hidden)return;
    heartbeatBusy=true;
    try{await request("/api/motion/heartbeat",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({mission_id:ownedMission,client_id:clientId})});}
    catch(error){ownedMission=null;log("控制连接未确认；任务将超时停止。"+explain(error.message));}
    finally{heartbeatBusy=false;}
  },400);
  setInterval(()=>{if(imageDeadline&&performance.now()>=imageDeadline)clearFrame();render();},100);
  document.addEventListener("visibilitychange",()=>{if(document.hidden){if(ownedMission&&token)motionAction("/api/motion/stop",{});ownedMission=null;clearFrame();lastState=telemetryDeadline=visionDeadline=0;}});

  // Preset path from the firmware snapshot, never a measured vehicle trace.
  // World coordinates = local route coordinates + start_world_mm (heading 0).
  const presetRoute={"source":"camera_udp_task6_candidate_20261102_120015","world_mm":[1900,1267],"points":[{"name":"S","world_mm":[130,1140]},{"name":"P1","world_mm":[820,920]},{"name":"P2","world_mm":[1110,780]},{"name":"P3","world_mm":[880,380]},{"name":"P4","world_mm":[1280,380]},{"name":"P5","world_mm":[1820,380]},{"name":"P6","world_mm":[690,110]},{"name":"G","world_mm":[60,40]}],"samples_mm":[[130,1140],[149,1134],[168,1128],[187,1122],[206,1116],[225,1110],[244,1104],[263,1097],[282,1091],[301,1085],[321,1079],[340,1073],[359,1067],[378,1061],[397,1055],[416,1049],[435,1043],[454,1037],[473,1031],[492,1025],[511,1018],[530,1012],[549,1006],[568,1000],[587,994],[606,988],[625,982],[644,976],[664,970],[683,964],[702,958],[721,952],[740,946],[759,940],[778,933],[797,927],[816,921],[834,913],[852,904],[870,896],[888,887],[906,878],[924,870],[942,861],[960,852],[978,844],[996,835],[1014,826],[1032,817],[1050,809],[1068,800],[1086,791],[1104,783],[1103,768],[1093,751],[1083,733],[1073,716],[1063,699],[1053,681],[1043,664],[1033,647],[1023,629],[1013,612],[1003,595],[993,577],[984,560],[974,543],[964,525],[954,508],[944,491],[934,473],[924,456],[914,439],[904,421],[894,404],[884,387],[892,380],[912,380],[932,380],[952,380],[972,380],[992,380],[1012,380],[1032,380],[1052,380],[1072,380],[1092,380],[1112,380],[1132,380],[1152,380],[1172,380],[1192,380],[1212,380],[1232,380],[1252,380],[1272,380],[1292,380],[1312,380],[1332,380],[1352,380],[1372,380],[1392,380],[1412,380],[1432,380],[1452,380],[1472,380],[1492,380],[1512,380],[1532,380],[1552,380],[1572,380],[1592,380],[1612,380],[1632,380],[1652,380],[1672,380],[1692,380],[1712,380],[1732,380],[1752,380],[1772,380],[1792,380],[1812,380],[1808,377],[1789,372],[1769,368],[1750,363],[1730,359],[1711,354],[1691,349],[1672,345],[1652,340],[1633,335],[1613,331],[1594,326],[1575,321],[1555,317],[1536,312],[1516,307],[1497,303],[1477,298],[1458,293],[1438,289],[1419,284],[1399,280],[1380,275],[1361,270],[1341,266],[1322,261],[1302,256],[1283,252],[1263,247],[1244,242],[1224,238],[1205,233],[1186,228],[1166,224],[1147,219],[1127,214],[1108,210],[1088,205],[1069,201],[1049,196],[1030,191],[1010,187],[991,182],[972,177],[952,173],[933,168],[913,163],[894,159],[874,154],[855,149],[835,145],[816,140],[796,135],[777,131],[758,126],[738,121],[719,117],[699,112],[680,109],[660,107],[640,104],[620,102],[600,100],[580,98],[560,96],[540,93],[521,91],[501,89],[481,87],[461,85],[441,82],[421,80],[401,78],[381,76],[361,73],[342,71],[322,69],[302,67],[282,65],[262,62],[242,60],[222,58],[202,56],[183,54],[163,51],[143,49],[123,47],[103,45],[83,43],[63,40],[60,40]],"length_mm":4213.562295123137,"dwell_seconds":4};
  const c=$("map").getContext("2d");
  function draw(){
    const scale=.35,left=67.5,top=45,[width,height]=presetRoute.world_mm;
    const screen=([x,y])=>[left+x*scale,top+(height-y)*scale];
    const line=(a,b)=>{c.beginPath();c.moveTo(...a);c.lineTo(...b);c.stroke();};
    c.clearRect(0,0,800,600);c.fillStyle="#141d25";c.fillRect(0,0,800,600);
    c.lineWidth=1;c.strokeStyle="#2b3d49";
    for(let x=0;x<=width;x+=100)line(screen([x,0]),screen([x,height]));
    for(let y=0;y<=height;y+=100)line(screen([0,y]),screen([width,y]));
    c.strokeStyle="#6b858e";c.strokeRect(left,top,width*scale,height*scale);
    c.font="13px Consolas, Microsoft YaHei";c.fillStyle="#aebfc9";c.textAlign="center";
    for(const x of [0,500,1000,1500,1900])c.fillText((x/1000).toFixed(2),screen([x,0])[0],514);
    c.textAlign="right";
    for(const y of [0,300,600,900,1267])c.fillText((y/1000).toFixed(3),left-10,screen([0,y])[1]+4);
    c.textAlign="left";c.fillText("Y / m",left-35,25);c.fillText("X / m",715,537);
    c.strokeStyle="#9ee1cc";c.lineWidth=3;c.lineJoin="round";c.beginPath();
    presetRoute.samples_mm.forEach((p,i)=>{const [x,y]=screen(p);if(i)c.lineTo(x,y);else c.moveTo(x,y);});c.stroke();
    for(let i=1;i<presetRoute.points.length;i++){
      const a=screen(presetRoute.points[i-1].world_mm),b=screen(presetRoute.points[i].world_mm);
      c.save();c.translate((a[0]+b[0])/2,(a[1]+b[1])/2);c.rotate(Math.atan2(b[1]-a[1],b[0]-a[0]));
      c.fillStyle="#9ee1cc";c.beginPath();c.moveTo(9,0);c.lineTo(-6,-5);c.lineTo(-6,5);c.closePath();c.fill();c.restore();
    }
    const position=state&&performance.now()-lastState<1200?state.telemetry?.position:null;
    if(position){
      c.save();c.beginPath();c.rect(left,top,width*scale,height*scale);c.clip();
      c.strokeStyle=position.available?"#ffb454":"#77828a";c.lineWidth=2;c.setLineDash([4,3]);c.beginPath();
      (position.trace||[]).forEach((p,i)=>{const [x,y]=screen([p[0]*1000,p[1]*1000]);if(i)c.lineTo(x,y);else c.moveTo(x,y);});c.stroke();c.setLineDash([]);
      if(position.available&&position.pose){
        const p=position.pose,[x,y]=screen([p.x*1000,p.y*1000]);c.translate(x,y);c.rotate(-p.yaw);
        c.fillStyle="#ffb454";c.beginPath();c.moveTo(12,0);c.lineTo(-8,-7);c.lineTo(-5,0);c.lineTo(-8,7);c.closePath();c.fill();
      }
      c.restore();
    }
    for(const p of presetRoute.points){
      const [x,y]=screen(p.world_mm);
      c.fillStyle=p.name==="S"?"#9ee1cc":p.name==="G"?"#f3a1a1":"#e3c88a";
      c.beginPath();c.arc(x,y,6,0,Math.PI*2);c.fill();
      c.font="bold 16px Consolas, Microsoft YaHei";c.textAlign=p.name==="P5"?"right":"left";
      c.fillText(p.name,x+(p.name==="P5"?-12:12),y-12);
    }
    c.textAlign="left";c.font="14px Microsoft YaHei";c.fillStyle="#aebfc9";
    c.fillText("S → P1 → P2 → P3 → P4 → P5 → P6 → G",67,560);
    c.font="12px Microsoft YaHei";c.fillText("预设路线 · P1–P6 各停留 4 s · G 停车",67,583);
  }
  $("route-length").textContent=(presetRoute.length_mm/1000).toFixed(2)+" m";
  $("route-points").replaceChildren();
  for(const p of presetRoute.points){
    const li=document.createElement("li");
    li.textContent=`${p.name} · (${p.world_mm.map(v=>(v/1000).toFixed(2)).join(", ")}) m`;
    $("route-points").append(li);
  }
  $("disconnect").textContent="断开并忘记配对";
  $("pair-code").placeholder="首次输入板端安装完成时显示的配对码";
  $("pair-hint").textContent="成功连接后在本浏览器记住配对；刷新或服务重启后自动重连。";
  draw();render();log("真实接入界面已加载。");
  const saved=remembered();if(saved)pair(saved,true);
})();
