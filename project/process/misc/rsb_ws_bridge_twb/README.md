# rsb_ws_bridge_claas

Displaying the RSB connection via websockets

## Verbindungsaufbau

### Scope Weiterleitung

Aktuell darf nur ein Client (Webbrowser) auf einen RSB scope subscriben.
Hierzu wird über WebSockets (WS) ein nativer String mit dem Scopenamen gesendet: /LASE.
Nach dem Empfang dieses Strings auf der Serverseite wird auf das erste Byte getestet "==/", und ein RSB-Listener subscribed auf den jeweiligen Scope und sendet die binary-blobs über WS an den Client weiter.
Nach dem Schließen der Seite auf dem Client wird serverseitig der Listener deallociert. 

### Logger Weiterleitung
Für die Weiterleitung der rsb-logger Ausgaben wird der Standardoutput über WS an dien Client gesendet.
Hierzu wird über WebSockets (WS) ein nativer String mit den Programmparametern gesendet: "--style monitor --scope /LASE".
Nach dem Empfang dieses Strings auf der Serverseite wird auf das erste Byte getestet "==-".
Der Server führt dann periodisch den Logger mit den Paramtern aus, und sendet die Standardausgabe unformatiert über die WS Verbindung zurück.
