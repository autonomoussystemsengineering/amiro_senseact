/*
 * Author: Markus Gaffke
 * File: disabled.js
 * Purpose: Disabling the right mouse click and mouse scroll
 * Last Changed: 05.07.2014
 *
 */



//Quelle: http://de.selfhtml.org/navigation/anzeige/rechte_maustaste.htm
function click (e) {
  if (!e)
    e = window.event;
  if ((e.type && e.type == "contextmenu") || (e.button && e.button == 2) || (e.which && e.which == 3)) {
    if (window.opera)
      window.alert("Sorry: Diese Funktion ist deaktiviert.");
    return false;
  }
}
if (document.layers)
  document.captureEvents(Event.MOUSEDOWN);
document.onmousedown = click;
document.oncontextmenu = click;

//Quelle: http://solidlystated.com/scripting/javascript-disable-mouse-wheel/
document.onmousewheel = function(){ stopWheel(); } /* IE7, IE8 */
if(document.addEventListener){ /* Chrome, Safari, Firefox */
    document.addEventListener('DOMMouseScroll', stopWheel, false);
}
 
function stopWheel(e){
    if(!e){ e = window.event; } /* IE7, IE8, Chrome, Safari */
    if(e.preventDefault) { e.preventDefault(); } /* Chrome, Safari, Firefox */
    e.returnValue = false; /* IE7, IE8 */
}


