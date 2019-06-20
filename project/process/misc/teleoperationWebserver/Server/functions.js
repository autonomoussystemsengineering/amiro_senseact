/*
 * Author: Markus Gaffke
 * File: functions.js
 * Purpose: different functions for all kinds
 * Last Changed: 05.07.2014
 *
 */

/*
*Converting InfraredData into a colorcode and returning this colorcode 
*
*/	
function getCode(sensorInt){
		//if(parseInt(sensorInt)>=3000){
		//if(parseInt(sensorInt)%3000>=3000){		//for testing porpuses
		//	return colorcode="ff0000";
		//}else{
           	//	var color_code = getGreenToRed(((parseInt(sensorInt)%3000)/3000)*100); //for testing porpuses
		//	var color_code = getGreenToRed((parseInt(sensorInt)/3000)*100);
	   		var color_code = getGreenToRed((parseInt(sensorInt));
			var code_split = color_code.split(",");		 	
	   		return  RGBtoHex(code_split[1], code_split[2], code_split[3]);  	
		//}        
	};


//http://stackoverflow.com/questions/7128675/from-green-to-red-color-depend-on-percentage
function getGreenToRed(percent){
           var g = percent<50 ? 255 : Math.floor(255-(percent*2-100)*255/100);
           var r = percent>50 ? 255 : Math.floor((percent*2)*255/100);
	return 'rgb(,'+r+','+g+',0)';  	
        };


//http://www.linuxtopia.org/online_books/javascript_guides/javascript_faq/rgbtohex.htm
function RGBtoHex(R,G,B) {return toHex(R)+toHex(G)+toHex(B)}
function toHex(N) {
 if (N==null) return "00";
 N=parseInt(N); if (N==0 || isNaN(N)) return "00";
 N=Math.max(0,N); N=Math.min(N,255); N=Math.round(N);
 return "0123456789ABCDEF".charAt((N-N%16)/16)
      + "0123456789ABCDEF".charAt(N%16);
};


/*
 * Getting a nice String for the Lights status
 *
 */
function getLightMessage(){
	var lightMessage;
	var white=" an /", red=" an /", green=" an /",blue=" an";	
	if(whiteLight == 0){white="aus /";}
	if(redLight == 0){red="aus /";}
	if(greenLight == 0){green="aus /";}
	if(blueLight == 0){blue="aus";}
	lightMessage="Licht: Weiß: " + white + " Rot: " + red + " Gruen: " + green + " Blau: " + blue;
		
	document.getElementById("Light_Status").textContent =lightMessage;
};	


/*
 * Getting the exact position of the mouse inside a canvas.
 *
 */
function getMousePos(canvas_control, evt) 
{
	var rect = canvas_control.getBoundingClientRect();
	return {x: evt.clientX - rect.left , y: evt.clientY - rect.top }

};

/*
 * Scalling the exact speed between the point where the mouse was pressed down and the moving position
 *
 */
function getSpeed(speedterm)
{
	var rangeLimit = 200;	// Sets the range to move inside the cameracanvas to reach 100% speed
	if(speedterm>rangeLimit){speedterm=rangeLimit}
	if(-speedterm>rangeLimit){speedterm=-rangeLimit}
	var actualSpeed=(speedterm/rangeLimit)*100;
	return parseInt(actualSpeed);
};

/*
 * Setting all images back to their original settings
 * 
 */
function setControlImgBack(){
	document.getElementById("img_OL").style.zIndex='-1';
	document.getElementById("img_O").style.zIndex='-2';
	document.getElementById("img_OR").style.zIndex='-3';
	document.getElementById("img_ML").style.zIndex='-4';
	document.getElementById("img_MR").style.zIndex='-5';
	document.getElementById("img_UL").style.zIndex='-6';
	document.getElementById("img_U").style.zIndex='-7';
	document.getElementById("img_UR").style.zIndex='-8';
	if(in_control){socket_camera.send(1099);} // send stop Signal
}

/*
 * Hidding all images after giving up on control
 * 
 */
function showControlpannel(){
		document.getElementById("imgnormal").style.visibility ="visible";
		document.getElementById("img_NO_CONTROL").style.visibility ="hidden";
		document.getElementById("img_OL").style.visibility ="visible";
		document.getElementById("img_O").style.visibility ="visible";
		document.getElementById("img_OR").style.visibility ="visible";
		document.getElementById("img_MR").style.visibility ="visible";
		document.getElementById("img_ML").style.visibility ="visible";
		document.getElementById("img_UR").style.visibility ="visible";
		document.getElementById("img_U").style.visibility ="visible";
		document.getElementById("img_UL").style.visibility ="visible";
};


