Test

CAREFUL !!! The CM_GS.bin that is not in specified dbm folder is 16dbm !!!
No transmission occured during 15dbm
>Packet receive and transmit perfectly at close distance but RSSI and SNR value is much higher than 
expected. There should be a lot of loss for the RSSI and SNR value.

Transmission between GS and SM working as intended (refer screenshot)
>Problem with UART1 Tx not receiving command from serial monitor solved by utilizing interrupt 
to save the input value into a var. So, the code is something like this; 
print : "Please input: "
while(isUserInput == false); //wait until user input is done / escape
print : userInput
>UART code from nucleo does not seem to work

RTC enable not yet implemented
>To retry soon