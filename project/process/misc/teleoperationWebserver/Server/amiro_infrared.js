/*
 * Author: Markus Gaffke
 * File: disabled.js
 * Purpose: Updating the colorcones of the amiro
 * Last Changed: 05.07.2014
 *
 */


function updateAmiro(infraredData){//for DEBUGGING purposses. if set to true all individual Infrared numbers are shown 								in statusbox 
			if(show_InfraredData)
			{document.getElementById("Ired_0").textContent = "Infrarot 1: " + infraredData[1];}
			colorcode = getCode(infraredData[4]);
			context1.fillStyle = "#"+colorcode;
			context1.fill();
			context1.stroke();
 			
			if(show_InfraredData)
			{document.getElementById("Ired_1").textContent = "Infrarot 2: " + infraredData[2];}	
			colorcode = getCode(infraredData[3]);
			context2.fillStyle = "#"+colorcode;
			context2.fill();
			context2.stroke();


 			if(show_InfraredData)
			{document.getElementById("Ired_2").textContent = "Infrarot 3: " + infraredData[3];}
			colorcode = getCode(infraredData[2]);
			context3.fillStyle = "#"+colorcode;
			context3.fill();
			context3.stroke();
			
 			if(show_InfraredData)
			{document.getElementById("Ired_3").textContent = "Infrarot 4: " + infraredData[4];}
			colorcode = getCode(infraredData[1]);
			context4.fillStyle = "#"+colorcode;
			context4.fill();
			context4.stroke();

 			if(show_InfraredData)
			{document.getElementById("Ired_4").textContent = "Infrarot 5: " + infraredData[5];}
			colorcode = getCode(infraredData[8]);
			context5.fillStyle = "#"+colorcode;
			context5.fill();
			context5.stroke();

			if(show_InfraredData)
			{document.getElementById("Ired_5").textContent = "Infrarot 6: " + infraredData[6];}		
			colorcode = getCode(infraredData[7]);
			context6.fillStyle = "#"+colorcode;
			context6.fill();
			context6.stroke();
	
 			if(show_InfraredData)
			{document.getElementById("Ired_6").textContent = "Infrarot 7: " + infraredData[7];}
			colorcode = getCode(infraredData[6]);
			context7.fillStyle = "#"+colorcode;
			context7.fill();
			context7.stroke();


 			if(show_InfraredData)
			{document.getElementById("Ired_7").textContent = "Infrarot 8: " + infraredData[8];}
			colorcode = getCode(infraredData[5]);
			context8.fillStyle = "#"+colorcode;
			context8.fill();
			context8.stroke();

};
