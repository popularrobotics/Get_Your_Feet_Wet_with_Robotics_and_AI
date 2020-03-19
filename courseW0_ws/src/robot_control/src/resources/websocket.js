var websocket=window.WebSocket || window.MozWebSocket; 
var isConnected = false;


//显示提示信息
function showDiagMsg(MsgFieldId,MsgText, diagPageId) {
  $("#dialog-msg").html(MsgText);
  $( "#dialog-confirm" ).dialog( "open" );
}

//处理打开webSocket
function doOpen(){
//  showDiagMsg("infoField","connected", "infoDialog");
     isConnected = true;
    //3:初始化地图并显示默认位置，发送登入消息
 	//4:每50秒定时刷新 						
// 	setInterval(doRefresh(mapArea), 50000);	
	$('#connect').val("connected");
	$("#connect").attr({"disabled":"disabled"});
}

//处理关闭webSocket
function doClose(){
	//连接断开
	showDiagMsg("infoField","connection lost", "infoDialog");
	isConnected = false;
	$('#connect').val("connect");
	$("#btnzhuce").removeAttr("disabled");
}

//处理webSocket Error
function doError() {
	showDiagMsg("infoField","connection error!", "infoDialog");
	isConnected = false;
	$('#connect').val("connect");
	$("#connect").removeAttr("disabled");
}

//处理websockett服务端返回(注意后台返回的message为json字符串)
function doMessage(message){
	var event = $.parseJSON(message.data);
	//doReciveEvent(event);
}

//处理发送消息(注意message是javascript Obj对象)
function doSend(message) {
	if (websocket != null) {
		websocket.send(JSON.stringify(message));
	} else {
		showDiagMsg("infoField","connection lost, can't communicate with the server!", "infoDialog");
	}
}

//初始话WebSocket
function initWebSocket(wcUrl) {
	if (window.WebSocket) {
		websocket = new WebSocket(encodeURI(wcUrl));
		websocket.onopen = doOpen;
		websocket.onerror = doError;
		websocket.onclose = doClose;
		websocket.onmessage = doMessage;
	}
	else{
		showDiagMsg("infoField","Your device doesn't support webSocket!", "infoDialog");
		
	}
};

