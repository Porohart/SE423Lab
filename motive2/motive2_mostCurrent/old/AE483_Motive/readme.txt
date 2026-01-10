Connect your WiFi to MechNight-5G (Password: f33dback5)
Open a terminal and run ifconfig to get your laptop IP connected to MechNight-5G

First edit server_receiving.py to change the line of code
UDP_IP =  "your lab pc or laptop IP connected to MechNight-5G"   example "192.168.1.140"
save this file

On motrack pc edit c:\AE483_Motive\motrack_AE483.py
edit first line of list of IPs to your PCs IP  example  address_list = ["192.168.1.140",
save file

launch Motive
Open Existing Project and select c:\motive_projects\AE483 2021-11-09 09.52.38 AM.ttp
Select your markers to create rigid body one
Select steaming pane and BroadCast Frame Data should be checked

Then go to Desktop first double click on the shortcut SampleClient.bat - Shortcut to run it

Again at the Desktop double click on the shortcut AE483PythonScript.bat - Shortcut to run it ... you may have to wait a minute for this to start printing out info

Now go back to your PC or laptop, open a terminal, change your directory path to where you have the server_receiving file saved, and run  python server_receiving.py  or on linux maybe use python3

