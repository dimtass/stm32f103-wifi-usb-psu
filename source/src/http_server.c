/*
 * http_server.c
 *
 *  Created on: 6 Apr 2017
 *      Author: dimtass
 */
#include "platform_config.h"
#include "http_server.h"
#include "esp8266.h"
#include "mcp41xxx.h"
#include "flash.h"

#define BTN_DECR_1		"btn01"
#define BTN_DECR_1_STR	"-"
#define BTN_INCR_1		"btn02"
#define BTN_INCR_1_STR	"+"
#define BTN_DECR_5		"btn03"
#define BTN_DECR_5_STR	"--"
#define BTN_INCR_5		"btn04"
#define BTN_INCR_5_STR	"++"
#define BTN_VOLT_1		"btn05"
#define BTN_VOLT_1_STR	"2V5"
#define BTN_VOLT_2		"btn06"
#define BTN_VOLT_2_STR	"3V3"
#define BTN_VOLT_3		"btn07"
#define BTN_VOLT_3_STR	"5V"
#define BTN_VOLT_4		"btn08"
#define BTN_VOLT_4_STR	"6V"
#define BTN_VOLT_5		"btn09"
#define BTN_VOLT_5_STR	"12V"
#define BTN_SAVE		"btn10"
#define BTN_SAVE_STR	"Save"
#define BTN_RELAY_ON	"btn11"
#define BTN_RELAY_ON_STR	"ON"
#define BTN_RELAY_OFF	"btn12"
#define BTN_RELAY_OFF_STR	"OFF"
/**
 * The html data
 */
char html_index[] = "<!DOCTYPE html>\n"
		"<html>\n"
		"<head>\n"
		"<style>\n"
		"[unselectable] {\n"
		    "-webkit-user-select: none;\n"
		    "-moz-user-select: none;\n"
		    "user-select: none;\n"
		"}\n"
		".mx-button input {\n"
		    "display: none;\n"
		"}\n"
		".mx-button label {\n"
		    "display:inline-block;\n"
			"float: left;\n"
		    "width:50px;\n"
		    "height:50px;\n"
		    "line-height:50px;\n"
		    "border: 2px solid #f5f5f5;\n"
		    "border-radius: 50%;\n"
		    "color: #f5f5f5;\n"
		    "padding: 5px 10px;\n"
		    "text-align:center;\n"
		    "text-decoration:none;\n"
		    "background: #464646;\n"
		    "box-shadow: 0 0 3px gray;\n"
		    "font-size:16px;\n"
		    "font-weight:bold;\n"
		"}\n"
		".mx-button label:hover {\n"
		    "background: #262626;\n"
		"}\n"
		".mx-button label:active,\n"
		".mx-button input:focus + label {\n"
		    "background: #73AD21;\n"
		"}\n"
		".mx-button input:checked + label {\n"
		    "background: #73AD21;\n"
		"}\n"
		".mx-button {\n"
			"display: inline;\n"
		"}\n"
		"h10 {\n"
		"font-size:9px;\n"
		"}\n"
		"</style>\n"
		"<script type=\"text/javascript\">\n"
			"function btn_press () {\n"
		        "var elements = document.getElementsByName(\"btn\");\n"
		        "for (var i=0; i < elements.length; i++) {\n"
		            "elements[i].onclick = function() {\n"
						"post(this.id);\n"
		            "}\n"
		        "}\n"
		    "}\n"
		"function post(param) {\n"
			"var xmlhttp = new XMLHttpRequest();\n"
			"xmlhttp.open(\"POST\", \"http://" HTTP_IP_ADDRESS "?btn=\" + param, true);\n"
			"xmlhttp.send();\n"
		"}\n"
		"</script>\n"
		"</head>\n"
		"<body>\n"
		"<h4>Power & Trim:</h4>\n"
		"<form>\n"
			"<div class=\"mx-button\">\n"
				"<input type=\"radio\" name=\"mx-pwr\" id=\"button11\">\n"
				"<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_RELAY_ON "\" for=\"button11\" unselectable>" BTN_RELAY_ON_STR "</label>\n"
			"</div>\n"
			"<div class=\"mx-button\">\n"
				"<input type=\"radio\" name=\"mx-pwr\" id=\"button12\">\n"
				"<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_RELAY_OFF "\" for=\"button12\" unselectable>" BTN_RELAY_OFF_STR "</label>\n"
			"</div>\n"
		    "<div class=\"mx-button\">\n"
		        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_DECR_1 "\" for=\"button1\" unselectable>" BTN_DECR_1_STR "</label>\n"
		    "</div>\n"
		    "<div class=\"mx-button\">\n"
		        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_INCR_1 "\" for=\"button2\" unselectable>" BTN_INCR_1_STR "</label>\n"
		    "</div>\n"
			"<div class=\"mx-button\">\n"
				"<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_DECR_5 "\" for=\"button3\" unselectable>" BTN_DECR_5_STR "</label>\n"
			"</div>\n"
			"<div class=\"mx-button\">\n"
				"<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_INCR_5 "\" for=\"button4\" unselectable>" BTN_INCR_5_STR "</label>\n"
			"</div>\n"
		"</form>\n"
		"<br><br><br>"
		"<h4>Predefined:</h4>"
		"<form>\n"
		    "<div class=\"mx-button\">\n"
		        "<input type=\"radio\" name=\"mx\" id=\"button5\">\n"
		        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_VOLT_1 "\" for=\"button5\" unselectable>" BTN_VOLT_1_STR "</label>\n"
		    "</div>\n"
		    "<div class=\"mx-button\">\n"
		        "<input type=\"radio\" name=\"mx\" id=\"button6\">\n"
		        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_VOLT_2 "\" for=\"button6\" unselectable>" BTN_VOLT_2_STR "</label>\n"
		    "</div>\n"
		    "<div class=\"mx-button\">\n"
		        "<input type=\"radio\" name=\"mx\" id=\"button7\">\n"
		        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_VOLT_3 "\" for=\"button7\" unselectable>" BTN_VOLT_3_STR "</label>\n"
		    "</div>\n"
		    "<div class=\"mx-button\">\n"
		        "<input type=\"radio\" name=\"mx\" id=\"button8\">\n"
		        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_VOLT_4 "\" for=\"button8\" unselectable>" BTN_VOLT_4_STR "</label>\n"
		    "</div>\n"
		    "<div class=\"mx-button\">\n"
		        "<input type=\"radio\" name=\"mx\" id=\"button9\">\n"
		        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_VOLT_5 "\" for=\"button9\" unselectable>" BTN_VOLT_5_STR "</label>\n"
		    "</div>\n"
			    "<div class=\"mx-button\">\n"
			        "<label name=\"btn\" onclick=\"btn_press()\" id=\"" BTN_SAVE "\" for=\"button10\" unselectable>" BTN_SAVE_STR "</label>\n"
			    "</div>\n"
		"</form>\n"
		"<br><br><br><br><br><br>\n"
		"<h10>WiFi/USB power supply v.1.0 by dimtass (2017)</h10>\n"
		"</body>\n"
		"</html>\n";

void http_handler(uint8_t mux_ch, const char * req, size_t req_size)
{
	if (!strncmp(req, "GET / HTTP", 10)) {

		TRACE(("sending...\n"));
		sATCIPSENDMultiple(mux_ch, (uint8_t*) html_index, strlen(html_index));
	}
	else if (!strncmp(req, "POST /?", 7)) {
		/* check for btn POST cmd */
		if (!strncmp(&req[7], "btn=", 4)) {
			char * btn_cmd = (char*) &req[7+4];
			/* check individual buttons */
			if (!strncmp(btn_cmd, BTN_DECR_1, 5)) {
				if (*glb.pot_value < 255) *glb.pot_value += 1;
			}
			else if (!strncmp(btn_cmd, BTN_INCR_1, 5)) {
				if (*glb.pot_value > 0) *glb.pot_value -= 1;
			}
			else if (!strncmp(btn_cmd, BTN_DECR_5, 5)) {
				if (*glb.pot_value < 250) *glb.pot_value += 5;
				else *glb.pot_value = 250;
			}
			else if (!strncmp(btn_cmd, BTN_INCR_5, 5)) {
				if (*glb.pot_value > 5) *glb.pot_value -= 5;
				else *glb.pot_value = 0;
			}
			else if (!strncmp(btn_cmd, BTN_VOLT_1, 5)) {
				glb.pot_value = &glb.conf.pot_values[0];
			}
			else if (!strncmp(btn_cmd, BTN_VOLT_2, 5)) {
				glb.pot_value = &glb.conf.pot_values[1];
			}
			else if (!strncmp(btn_cmd, BTN_VOLT_3, 5)) {
				glb.pot_value = &glb.conf.pot_values[2];
			}
			else if (!strncmp(btn_cmd, BTN_VOLT_4, 5)) {
				glb.pot_value = &glb.conf.pot_values[3];;
			}
			else if (!strncmp(btn_cmd, BTN_VOLT_5, 5)) {
				glb.pot_value = &glb.conf.pot_values[4];;
			}
			else if (!strncmp(btn_cmd, BTN_RELAY_ON, 5)) {
				TRACE(("Relay ON\n"));
				GPIO_PORT_RELAY->ODR |= GPIO_PIN_RELAY;
			}
			else if (!strncmp(btn_cmd, BTN_RELAY_OFF, 5)) {
				TRACE(("Relay OFF\n"));
				GPIO_PORT_RELAY->ODR &= ~GPIO_PIN_RELAY;
			}
			else if (!strncmp(btn_cmd, BTN_SAVE, 5)) {
				/* save configuration in flash */
				TRACE(("Saving configuration to flash\n"));
				FLASH_write((uint8_t*) &glb.conf, sizeof(tp_conf));
			}
			mcp41xxx_set_value(1, *glb.pot_value);
			TRACE(("Setting pot to: %d\n", *glb.pot_value));
		}
		/* parse POST request */
	}
}
