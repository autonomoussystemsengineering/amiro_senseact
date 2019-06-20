/*
 * Author: Markus Gaffke
 * File: Steuern_Camera.js
 * Purpose: holding all eventlisteners for camera canvas
 * Last Changed: 05.07.2014
 *
 */

var canvas_Help = document.getElementById('help_Example');

canvas_Help.addEventListener('mousedown', function(ef) 
	{
		document.getElementById("help_Example").style.visibility ="hidden";
		document.getElementById("help_div").style.visibility ="hidden";
		document.getElementById("img_Help").style.visibility ="hidden";
	}, false);

