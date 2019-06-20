/*
 * Author: Markus Gaffke
 * File: draw.js
 * Purpose: Drawing the colorcones of the robot
 * Last Changed: 05.07.2014
 *
 */

//draws the BeBot InfraredSensors
//=================================================================canvas_toplefttlc=========================================================
 var canvas_toplefttlc = document.getElementById('toplefttlc');
      var context_toplefttlc = canvas_toplefttlc.getContext('2d');
  
	context_toplefttlc.beginPath();
	context_toplefttlc.lineWidth = 1;
	context_toplefttlc.moveTo(2, 47);
	context_toplefttlc.lineTo(17, 0);
	context_toplefttlc.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
	context_toplefttlc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
	context_toplefttlc.fill();
	context_toplefttlc.fillStyle = "#"+colorcode;
	context_toplefttlc.fill();
	context_toplefttlc.rotate(20*Math.PI/90);	
	context_toplefttlc.stroke();

//=================================================================canvas_topleftblc=========================================================
 var canvas_topleftblc = document.getElementById('topleftblc');
      var context_topleftblc = canvas_topleftblc.getContext('2d');
  
     	context_topleftblc.beginPath();
     	context_topleftblc.lineWidth = 1;
     	context_topleftblc.moveTo(2, 47);
     	context_topleftblc.lineTo(17, 0);
     	context_topleftblc.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_topleftblc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_topleftblc.fill();

	context_topleftblc.fillStyle = "#"+colorcode;
	context_topleftblc.fill();
	context_topleftblc.rotate(20*Math.PI/90);	
	context_topleftblc.stroke();

//=================================================================canvas_toplefttrc=========================================================
 var canvas_toplefttrc = document.getElementById('toplefttrc');
      var context_toplefttrc = canvas_toplefttrc.getContext('2d');
  
     	context_toplefttrc.beginPath();
     	context_toplefttrc.lineWidth = 1;
     	context_toplefttrc.moveTo(2, 47);
     	context_toplefttrc.lineTo(17, 0);
     	context_toplefttrc.lineTo(32, 47);
      //context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_toplefttrc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_toplefttrc.fill();

	context_toplefttrc.fillStyle = "#"+colorcode;
	context_toplefttrc.fill();
	context_toplefttrc.rotate(20*Math.PI/90);	
	context_toplefttrc.stroke();





//=================================================================canvas_bottomlefttlc======================================================
 var canvas_bottomlefttlc = document.getElementById('bottomlefttlc');
      var context_bottomlefttlc = canvas_bottomlefttlc.getContext('2d');
  
	context_bottomlefttlc.beginPath();
	context_bottomlefttlc.lineWidth = 1;
	context_bottomlefttlc.moveTo(2, 47);
	context_bottomlefttlc.lineTo(17, 0);
	context_bottomlefttlc.lineTo(32, 47);
      	//context_bottomlefttlc.arc(center-x,center-y,radius,start-angle,end-angle);
	context_bottomlefttlc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
	context_bottomlefttlc.fill();

	context_bottomlefttlc.fillStyle = "#"+colorcode;
	context_bottomlefttlc.fill();
	context_bottomlefttlc.rotate(20*Math.PI/90);	
	context_bottomlefttlc.stroke();

//=================================================================canvas_bottomleftblc======================================================
 var canvas_bottomleftblc = document.getElementById('bottomleftblc');
      var context_bottomleftblc = canvas_bottomleftblc.getContext('2d');
  
     	context_bottomleftblc.beginPath();
     	context_bottomleftblc.lineWidth = 1;
     	context_bottomleftblc.moveTo(2, 47);
     	context_bottomleftblc.lineTo(17, 0);
     	context_bottomleftblc.lineTo(32, 47);
      	//context_toprighttlc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_bottomleftblc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_bottomleftblc.fill();

	context_bottomleftblc.fillStyle = "#"+colorcode;
	context_bottomleftblc.fill();
	context_bottomleftblc.rotate(20*Math.PI/90);	
	context_bottomleftblc.stroke();

//=================================================================canvas_bottomleftbrc======================================================
 var canvas_bottomleftbrc = document.getElementById('bottomleftbrc');
      var context_bottomleftbrc = canvas_bottomleftbrc.getContext('2d');
  
     	context_bottomleftbrc.beginPath();
     	context_bottomleftbrc.lineWidth = 1;
     	context_bottomleftbrc.moveTo(2, 47);
     	context_bottomleftbrc.lineTo(17, 0);
     	context_bottomleftbrc.lineTo(32, 47);
      	//context_bottomleftbrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_bottomleftbrc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_bottomleftbrc.fill();

	context_bottomleftbrc.fillStyle = "#"+colorcode;
	context_bottomleftbrc.fill();
	context_bottomleftbrc.rotate(20*Math.PI/90);	
	context_bottomleftbrc.stroke();




//=================================================================canvas_toprighttlc========================================================
 var canvas_toprighttlc = document.getElementById('toprighttlc');
      var context_toprighttlc = canvas_toprighttlc.getContext('2d');
  
	context_toprighttlc.beginPath();
	context_toprighttlc.lineWidth = 1;
	context_toprighttlc.moveTo(2, 47);
	context_toprighttlc.lineTo(17, 0);
	context_toprighttlc.lineTo(32, 47);
      	//context_toprighttlc.arc(center-x,center-y,radius,start-angle,end-angle);
	context_toprighttlc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
	context_toprighttlc.fill();

	context_toprighttlc.fillStyle = "#"+colorcode;
	context_toprighttlc.fill();
	context_toprighttlc.rotate(20*Math.PI/90);	
	context_toprighttlc.stroke();

//=================================================================canvas_toprighttrc========================================================
 var canvas_toprighttrc = document.getElementById('toprighttrc');
      var context_toprighttrc = canvas_toprighttrc.getContext('2d');
  
     	context_toprighttrc.beginPath();
     	context_toprighttrc.lineWidth = 1;
     	context_toprighttrc.moveTo(2, 47);
     	context_toprighttrc.lineTo(17, 0);
     	context_toprighttrc.lineTo(32, 47);
      	//context_toprighttlc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_toprighttrc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_toprighttrc.fill();
	context_toprighttrc.fillStyle = "#"+colorcode;
	context_toprighttrc.fill();
	context_toprighttrc.rotate(20*Math.PI/90);	
	context_toprighttrc.stroke();

//=================================================================canvas_toprightbrc========================================================
 var canvas_toprightbrc = document.getElementById('toprightbrc');
      var context_toprightbrc = canvas_toprightbrc.getContext('2d');
  
     	context_toprightbrc.beginPath();
     	context_toprightbrc.lineWidth = 1;
     	context_toprightbrc.moveTo(2, 47);
     	context_toprightbrc.lineTo(17, 0);
     	context_toprightbrc.lineTo(32, 47);
      	//context_toprightbrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_toprightbrc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_toprightbrc.fill();
	context_toprightbrc.fillStyle = "#"+colorcode;
	context_toprightbrc.fill();
	context_toprightbrc.rotate(20*Math.PI/90);	
	context_toprightbrc.stroke();





//=================================================================canvas_bottomrighttrc=====================================================
 var canvas_bottomrighttrc = document.getElementById('bottomrighttrc');
      var context_bottomrighttrc = canvas_bottomrighttrc.getContext('2d');
  
	context_bottomrighttrc.beginPath();
	context_bottomrighttrc.lineWidth = 1;
	context_bottomrighttrc.moveTo(2, 47);
	context_bottomrighttrc.lineTo(17, 0);
	context_bottomrighttrc.lineTo(32, 47);
      	//context_bottomrighttrc.arc(center-x,center-y,radius,start-angle,end-angle);
	context_bottomrighttrc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
	context_bottomrighttrc.fill();

	context_bottomrighttrc.fillStyle = "#"+colorcode;
	context_bottomrighttrc.fill();
	context_bottomrighttrc.rotate(20*Math.PI/90);	
	context_bottomrighttrc.stroke();

//=================================================================canvas_bottomrightblc=====================================================
 var canvas_bottomrightblc = document.getElementById('bottomrightblc');
      var context_bottomrightblc = canvas_bottomrightblc.getContext('2d');
  
     	context_bottomrightblc.beginPath();
     	context_bottomrightblc.lineWidth = 1;
     	context_bottomrightblc.moveTo(2, 47);
     	context_bottomrightblc.lineTo(17, 0);
     	context_bottomrightblc.lineTo(32, 47);
      	//context_toprighttlc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_bottomrightblc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_bottomrightblc.fill();

	context_bottomrightblc.fillStyle = "#"+colorcode;
	context_bottomrightblc.fill();
	context_bottomrightblc.rotate(20*Math.PI/90);	
	context_bottomrightblc.stroke();

//=================================================================canvas_bottomrightbrc=====================================================
 var canvas_bottomrightbrc = document.getElementById('bottomrightbrc');
      var context_bottomrightbrc = canvas_bottomrightbrc.getContext('2d');
  
     	context_bottomrightbrc.beginPath();
     	context_bottomrightbrc.lineWidth = 1;
     	context_bottomrightbrc.moveTo(2, 47);
     	context_bottomrightbrc.lineTo(17, 0);
     	context_bottomrightbrc.lineTo(32, 47);
      	//context_bottomrightbrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context_bottomrightbrc.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context_bottomrightbrc.fill();

	context_bottomrightbrc.fillStyle = "#"+colorcode;
	context_bottomrightbrc.fill();
	context_bottomrightbrc.rotate(20*Math.PI/90);	
	context_bottomrightbrc.stroke();

//draws the amiro InfraredSensors
//===========================================================canvas Amiro=======================================================
var canvas_amiro1 = document.getElementById('canvas_amiro1');
      var context1 = canvas_amiro1.getContext('2d');
  
     	context1.beginPath();
     	context1.lineWidth = 1;
     	context1.moveTo(2, 47);
     	context1.lineTo(17, 0);
     	context1.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context1.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context1.fill();
	context1.fillStyle = "#"+colorcode;
	context1.fill();
	context1.rotate(20*Math.PI/90);	
	context1.stroke();


var canvas_amiro2 = document.getElementById('canvas_amiro2');
      var context2 = canvas_amiro2.getContext('2d');
  
     	context2.beginPath();
     	context2.lineWidth = 1;
     	context2.moveTo(2, 47);
     	context2.lineTo(17, 0);
     	context2.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context2.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context2.fill();
	context2.fillStyle = "#"+colorcode;
	context2.fill();
	context2.rotate(20*Math.PI/90);	
	context2.stroke();

var canvas_amiro3 = document.getElementById('canvas_amiro3');
      var context3 = canvas_amiro3.getContext('2d');
  
     	context3.beginPath();
     	context3.lineWidth = 1;
     	context3.moveTo(2, 47);
     	context3.lineTo(17, 0);
     	context3.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context3.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context3.fill();
	context3.fillStyle = "#"+colorcode;
	context3.fill();
	context3.rotate(20*Math.PI/90);	
	context3.stroke();

var canvas_amiro4 = document.getElementById('canvas_amiro4');
      var context4 = canvas_amiro4.getContext('2d');
  
     	context4.beginPath();
     	context4.lineWidth = 1;
     	context4.moveTo(2, 47);
     	context4.lineTo(17, 0);
     	context4.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context4.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context4.fill();
	context4.fillStyle = "#"+colorcode;
	context4.fill();
	context4.rotate(20*Math.PI/90);	
	context4.stroke();

var canvas_amiro5 = document.getElementById('canvas_amiro5');
      var context5 = canvas_amiro5.getContext('2d');
  
     	context5.beginPath();
     	context5.lineWidth = 1;
     	context5.moveTo(2, 47);
     	context5.lineTo(17, 0);
     	context5.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context5.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context5.fill();
	context5.fillStyle = "#"+colorcode;
	context5.fill();
	context5.rotate(20*Math.PI/90);	
	context5.stroke();

var canvas_amiro6 = document.getElementById('canvas_amiro6');
      var context6 = canvas_amiro6.getContext('2d');
  
     	context6.beginPath();
     	context6.lineWidth = 1;
     	context6.moveTo(2, 47);
     	context6.lineTo(17, 0);
     	context6.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context6.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context6.fill();
	context6.fillStyle = "#"+colorcode;
	context6.fill();
	context6.rotate(20*Math.PI/90);	
	context6.stroke();

var canvas_amiro7 = document.getElementById('canvas_amiro7');
      var context7 = canvas_amiro7.getContext('2d');
  
     	context7.beginPath();
     	context7.lineWidth = 1;
     	context7.moveTo(2, 47);
     	context7.lineTo(17, 0);
     	context7.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context7.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context7.fill();
	context7.fillStyle = "#"+colorcode;
	context7.fill();
	context7.rotate(20*Math.PI/90);	
	context7.stroke();

var canvas_amiro8 = document.getElementById('canvas_amiro8');
      var context8 = canvas_amiro8.getContext('2d');
  
     	context8.beginPath();
     	context8.lineWidth = 1;
     	context8.moveTo(2, 47);
     	context8.lineTo(17, 0);
     	context8.lineTo(32, 47);
	//context_toplefttrc.arc(center-x,center-y,radius,start-angle,end-angle);
     	context8.arc(17,0,48,0.425*Math.PI,0.59*Math.PI);
     	context8.fill();
	context8.fillStyle = "#"+colorcode;
	context8.fill();
	context8.rotate(20*Math.PI/90);	
	context8.stroke();

