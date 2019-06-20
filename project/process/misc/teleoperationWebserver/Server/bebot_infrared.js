/*
 * Author: Markus Gaffke
 * File: bebot_infrared.js
 * Purpose: Updating the colorcones of the bebot
 * Last Changed: 05.07.2014
 *
 */

function updateBeBot(infraredData){//for DEBUGGING purposses. if set to true all individual Infrared numbers are shown 								in statusbox 
			if(show_InfraredData){document.getElementById("Ired_0").textContent = "Infrarot 1: " + infraredData[1];}
			colorcode = getCode(infraredData[1]);
			context_toplefttrc.fillStyle = "#"+colorcode;
			context_toplefttrc.fill();
			context_toplefttrc.stroke();
 			
			if(show_InfraredData){document.getElementById("Ired_1").textContent = "Infrarot 2: " + infraredData[2];}	
			colorcode = getCode(infraredData[2]);
			context_toplefttlc.fillStyle = "#"+colorcode;
			context_toplefttlc.fill();
			context_toplefttlc.stroke();


 			if(show_InfraredData){document.getElementById("Ired_2").textContent = "Infrarot 3: " + infraredData[3];}
			colorcode = getCode(infraredData[3]);
			context_topleftblc.fillStyle = "#"+colorcode;
			context_topleftblc.fill();
			context_topleftblc.stroke();
			
 			if(show_InfraredData){document.getElementById("Ired_3").textContent = "Infrarot 4: " + infraredData[4];}
			colorcode = getCode(infraredData[4]);
			context_bottomlefttlc.fillStyle = "#"+colorcode;
			context_bottomlefttlc.fill();
			context_bottomlefttlc.stroke();

 			if(show_InfraredData){document.getElementById("Ired_4").textContent = "Infrarot 5: " + infraredData[5];}
			colorcode = getCode(infraredData[5]);
			context_bottomleftblc.fillStyle = "#"+colorcode;
			context_bottomleftblc.fill();
			context_bottomleftblc.stroke();

			if(show_InfraredData){document.getElementById("Ired_5").textContent = "Infrarot 6: " + infraredData[6];}	
			colorcode = getCode(infraredData[6]);
			context_bottomleftbrc.fillStyle = "#"+colorcode;
			context_bottomleftbrc.fill();
			context_bottomleftbrc.stroke();
	
 			if(show_InfraredData){document.getElementById("Ired_6").textContent = "Infrarot 7: " + infraredData[7];}
			colorcode = getCode(infraredData[7]);
			context_bottomrightblc.fillStyle = "#"+colorcode;
			context_bottomrightblc.fill();
			context_bottomrightblc.stroke();


 			if(show_InfraredData)
			{document.getElementById("Ired_7").textContent = "Infrarot 8: " + infraredData[8];}		
			colorcode = getCode(infraredData[8]);
			context_bottomrightbrc.fillStyle = "#"+colorcode;
			context_bottomrightbrc.fill();
			context_bottomrightbrc.stroke();

 			if(show_InfraredData)
			{document.getElementById("Ired_8").textContent = "Infrarot 9: " + infraredData[9];}
			colorcode = getCode(infraredData[9]);
			context_bottomrighttrc.fillStyle = "#"+colorcode;
			context_bottomrighttrc.fill();
			context_bottomrighttrc.stroke();

 			if(show_InfraredData)
			{document.getElementById("Ired_9").textContent = "Infrarot 10: " + infraredData[10];}
			colorcode = getCode(infraredData[10]);
			context_toprightbrc.fillStyle = "#"+colorcode;
			context_toprightbrc.fill();
			context_toprightbrc.stroke();
			
			if(show_InfraredData)
			{document.getElementById("Ired_10").textContent = "Infrarot 11: " + infraredData[11];}
			colorcode = getCode(infraredData[11]);
			context_toprighttrc.fillStyle = "#"+colorcode;
			context_toprighttrc.fill();
			context_toprighttrc.stroke();
		
 			if(show_InfraredData)
			{document.getElementById("Ired_11").textContent = "Infrarot 12: "+infraredData[12];}
			colorcode = getCode(infraredData[12]);			
			context_toprighttlc.fillStyle = "#"+colorcode;
			context_toprighttlc.fill();
			context_toprighttlc.stroke();
};


