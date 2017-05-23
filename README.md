# LoCoMo

Note: To upload to LoCoMo boards, need to install Simon Peter's ESP8266 library from the Arduino Library Manager. For your board select the WeMos mini. CHECK to make sure you have the proper CH340G driver installed on your machine, a bad driver caused a kernel panic on my mac once. To use test.py install pygame http://www.pygame.org/download.shtml.

Serial Instructions:
Use 115200 and terminate with a newline character. Supported commands are "stop", "brake x", "pow x t", "pos x t", and "log x", where x is the argument and t is the timeout.