// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------

#include <WiFi.h>

/***********html&javascript网页构建**********/
const char basic_web[] PROGMEM = R"=====(

<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>legged wheel robot web ctrl</title>
  <style>
    h2{
        width: auto;
        height: 60px;
        line-height: 60px;
        text-align: center;
        font-family: 等线;
        color: white;
        background-color:cornflowerblue;
        border-radius: 12px;
    }
    input{
        width: 160px;
        height: 30px;
        margin: 0px;
    }
    .sliderLabel{
        float: left;
        text-align: center;
        line-height: 30px;
        height: 30px;  
        /* border: 2px solid red; */
        width: 200px;
    }

    .sliders{
        width: 400px;
        height: 150px;
        /* border: 2px red solid; */
        margin: 10px auto;
        position: relative;
    }

    .selects{
        width: 300px;
        height: 50px;
        padding: 10px;
        margin: 10px auto;
        /* border: 2px red solid; */
        position: relative;
        border-radius: 10px;

    }
    .view {
        width: 250px;
        height: 50px;
        padding: 0px;
        margin: 20px auto;
        /* border: 2px red solid; */
    }
    .view2 {
        width: 140px;
        height: 30px;
        padding: 0px;
        margin: 15px auto;
        vertical-align:middle;
        /* border: 2px red solid; */
    }
    .btn1 {
        width: 80px;
        height: 40px;
        padding: 0px;
        margin: 0px;
        border-radius: 10px;
        background-color: white;
        display: inline-block;
    }
    form {
        width: 80px;
        height: 40px;
        padding: 0px;
        margin: 10px auto;
        display: inline-block;
    }
    .buttons{
        width: 300px;
        height: 180px;
        padding: 10px;
        margin: 10px auto;
        position: relative;
        /* border: 2px red solid; */
    }
    .dir{
        font-size: 15px;
        /* font-family: 'Courier New', Courier, monospace; */
        /* font-weight: bold; */
        width: 100px;
        height: 60px;
        text-align: center;
        border-radius: 12px;
        background-color: white;
        color: cornflowerblue;
        border: 3px solid cornflowerblue;
        padding: 0px;
        transition: all 0.3s;
    }
    option {
        text-align: center;
        font-size: 15px;
    }

    #reset {
        width: 100px;
        height: 40px;
        text-align: center;
        border-radius: 12px;
        border: 2px solid;
        font-size: 15px;
        display: inline-block;
        position: absolute;
        top: 10px;
        left: 0px;
        /* 上 右 下 左 */
    }
    #gait_select {
        width: 100px;
        height: 40px;
        border: 2px solid red;
        font-size: 15px;
        color: red;
        border-radius: 12px;
        display: inline-block;
        position: absolute;
        top: 10px;
        left: 110px;
    }
    #gait_submit {
        width: 100px;
        height: 40px;
        text-align: center;
        border: 2px solid red;
        color: red;
        font-size: 15px;
        display: inline-block;
        background-color: white;
        border-radius: 12px;
        position: absolute;
        top: 10px;
        left: 220px;
    }
    #forward{
        display: inline-block;
        position: absolute;
        top: 0px;
        left: 110px;
    }
    #back{
        display: inline-block;
        position: absolute;
        left: 110px;
        top: 140px;
    }
    #jump{
        display: inline-block;
        position: absolute;
        top: 70px;
        left: 110px;
    }
    #right{
        display: inline-block;
        position: absolute;
        top: 70px;
        left: 220px;
    }
    #left{
        display: inline-block;
        position: absolute;
        top: 70px;
        left: 0px;
    }




    
    /* Switch开关样式 */
    /* 必须是input为 checkbox class 添加 switch 才能实现以下效果 */
    input[type='checkbox'].switch{
        outline: none;
        appearance: none;
        -webkit-appearance: none;
        -moz-appearance: none;
        position: relative;
        width: 40px;
        height: 20px;
        background: #ccc;
        border-radius: 10px;
        transition: border-color .3s, background-color .3s;
        margin: 0px 20px 0px 0px;
    }

    input[type='checkbox'].switch::after {
        content: '';
        display: inline-block;
        width: 1rem;
        height:1rem;
        border-radius: 50%;
        background: #fff;
        box-shadow: 0, 0, 2px, #999;
        transition:.4s;
        top: 2px;
        position: absolute;
        left: 2px;
    }

    input[type='checkbox'].switch:checked {
        background: rgb(78, 78, 240);
    }
    /* 当input[type=checkbox]被选中时：伪元素显示下面样式 位置发生变化 */
    input[type='checkbox'].switch:checked::after {
        content: '';
        position: absolute;
        left: 55%;
        top: 2px;
    }
    *{
        -webkit-touch-callout:none; 
        -webkit-user-select:none; 
        -khtml-user-select:none; 
        -moz-user-select:none;
        -ms-user-select:none; 
        user-select:none;
    }
    
    /*摇杆内容*/
    .row
    {
    display: inline-flex;
    clear: both;
    }
    .columnLateral
    {
    float: left;
    width: 15%;
    min-width: 300px;
    }

    #joystick
    {
    border: 0px solid #FF0000;
    }

    .dimSlide { transform: scaleX(2) rotate(0deg); width: 200px;position: relative}
        
  </style>
</head>

<!-- onload 事件在页面载入完成后立即触发 -->
<body onload="javascript:socket_init()">
    <h2>WL-PRO WiFi遥控模式</h2>
    <div class="view2">
        <input type="checkbox" id="stable" class="switch" onclick="is_stable()" style="vertical-align:middle">Robot Go!</input>
    </div>
    <center><body>
        <!-- Example of two JoyStick integrated in the page structure -->
        <div class="row">
           <div class="columnLateral">
            <div id="joy1Div" style="width:200px;height:200px;margin:10px"></div>
            
            <!-- angular_vel=<input id="joy1X" type="text" style="border-style:none;width:30px;" /> -->
            <!-- linear_vel=<input id="joy1Y" type="text" style="border-style:none;width:30px;" /> -->
          </div>
    </center>
    <div class="sliders">
        <div>
            <input type="range" min="32" max="85" value="38" id="hSlider" oninput="setHeight()" />
            <label class="sliderLabel" for="hSlider" id="hLabel">BaseHeight: 38mm</label>
        </div>
        <div>
            <input type="range" min="-30" max="30" value="0" id="rollSlider" oninput="setroll()" />
            <label class="sliderLabel" for="rollSlider" id="rollLabel">Roll: 0°</label>
        </div>
        
        <div>
            <input type="range" min="-200" max="200" value="0" id="linearSlider" oninput="setLinear()" />
            <label class="sliderLabel" for="linearSlider" id="linearLabel">LinearVel: 0mm/s</label>
        </div>
        <div>
            <input type="range" min="-100" max="100" value="0" id="angularSlider" oninput="setAngular()" />
            <label class="sliderLabel" for="angularSlider" id="angularLabel">AngularVel: 0°/s</label>
        </div>
    </div>
    
    <div class="buttons">
        <button class="dir" id="forward">Forward</button>
        <button class="dir" id="back">Back</button>
        <button class="dir" id="left">Left</button>
        <button class="dir" id="right">Right</button>
        <button class="dir" id="jump">Jump</button>
    </div>
   

    <!-- <p id="tips" align="center" style="font-size: 20;"></p> -->
    <script>
      
        var socket; // socket通信
        var g_roll=0; g_h=38; 
        var g_linear = 0; g_angular = 0; g_stable = 0; 
        var joyX = 0;
        var joyY = 0;
        // socket_init在页面载入完成后触发
        function socket_init() {
            // 初始化websocket客户端
            //socket = new WebSocket('ws://' + '192.168.4.1' + ':81/'); // AP模式
            socket = new WebSocket('ws://' + window.location.hostname + ':81/'); // sta模式
        }

        function setroll() {
            val = document.getElementById("rollSlider").value;
            val = parseInt(val);
            document.getElementById("rollLabel").innerHTML = "Roll: " + val + "°";
            g_roll = val;
            send_data();
        }  
        function setHeight() {
            val = document.getElementById("hSlider").value;
            val = parseInt(val);
            document.getElementById("hLabel").innerHTML = "BaseHeight: " + val + "mm";
            g_h = val;
            send_data();
        }    
       
        function setLinear() {
            val = document.getElementById("linearSlider").value;
            val = parseInt(val);
            document.getElementById("linearLabel").innerHTML = "LinearVel: " + val + "mm/s";
            g_linear = val;
            send_data();
        }
        function setAngular() {
            val = document.getElementById("angularSlider").value;
            val = parseInt(val);
            document.getElementById("angularLabel").innerHTML = "AngularVel: " + val + "°/s";
            g_angular = val;
            send_data();
        }

        
        function send_data() {
            var data = {'roll':g_roll,'height':g_h,
                        'linear':g_linear,'angular':g_angular,'stable':g_stable,
                        'mode':'basic','dir':"stop",
                        'joy_y':joyY,'joy_x':joyX,};
            socket.send(JSON.stringify(data));
            // console.log(data);
        }
        function is_stable() {
            var obj = document.getElementById("stable");
            if(obj.checked) {
                // alert("is_stable checked");
                g_stable = 1;
            }else {
                // alert("is_stable unchecked");
                g_stable = 0;
            }
            send_data();
        }

        var buttons = document.getElementsByClassName("dir");
        for(i=0;i<buttons.length;i++) {
            buttons[i].addEventListener("mousedown",move,true);
            buttons[i].addEventListener("mouseup",stop,true);
            buttons[i].addEventListener("touchstart",move,true);
            buttons[i].addEventListener("touchend",stop,true);
        }
        function move() {
            this.style = "background-color: cornflowerblue; color: white;";
            var data = {'dir':this.id,'mode':'basic',
                        'roll':g_roll,'height':g_h,
                        'linear':g_linear,'angular':g_angular,'stable':g_stable,
                        'joy_x':joyX,'joy_y':joyY,};
            socket.send(JSON.stringify(data));
            // console.log(data);
        }
        function stop() {
            this.style = "background-color: white; color: cornflowerblue;";
            var data = {'dir':"stop",'mode':'basic',
                        'roll':g_roll,'height':g_h,
                        'linear':g_linear,'angular':g_angular,'stable':g_stable,
                        'joy_x':joyX,'joy_y':joyY,};
            socket.send(JSON.stringify(data));
            // console.log(data); // 打印测试
        }
        

        /*摇杆内容*/
        var JoyStick = (function(container, parameters)
        {
            parameters = parameters || {};
            var title = (typeof parameters.title === "undefined" ? "joystick" : parameters.title),
                width = (typeof parameters.width === "undefined" ? 0 : parameters.width),
                height = (typeof parameters.height === "undefined" ? 0 : parameters.height),
                internalFillColor = (typeof parameters.internalFillColor === "undefined" ? "#00979C" : parameters.internalFillColor),
                internalLineWidth = (typeof parameters.internalLineWidth === "undefined" ? 2 : parameters.internalLineWidth),
                internalStrokeColor = (typeof parameters.internalStrokeColor === "undefined" ? "#00979C" : parameters.internalStrokeColor),
                externalLineWidth = (typeof parameters.externalLineWidth === "undefined" ? 2 : parameters.externalLineWidth),
                externalStrokeColor = (typeof parameters.externalStrokeColor ===  "undefined" ? "#0097BC" : parameters.externalStrokeColor),
                autoReturnToCenter = (typeof parameters.autoReturnToCenter === "undefined" ? true : parameters.autoReturnToCenter);
            
            // Create Canvas element and add it in the Container object
            var objContainer = document.getElementById(container);
            var canvas = document.createElement("canvas");
            canvas.id = title;
            if(width === 0) { width = objContainer.clientWidth; }
            if(height === 0) { height = objContainer.clientHeight; }
            canvas.width = width;
            canvas.height = height;
            objContainer.appendChild(canvas);
            var context=canvas.getContext("2d");
            
            var isPressing = 0;
            var isMoving = 0;
            var isRelease = 0;
            
            var circumference = 2 * Math.PI;
            var internalRadius = (canvas.width-((canvas.width/2)+10))/2;
            var maxMoveStick = internalRadius + 5;
            var externalRadius = internalRadius + 30;
            var centerX = canvas.width / 2;
            var centerY = canvas.height / 2;
            var directionHorizontalLimitPos = canvas.width / 10;
            var directionHorizontalLimitNeg = directionHorizontalLimitPos * -1;
            var directionVerticalLimitPos = canvas.height / 10;
            var directionVerticalLimitNeg = directionVerticalLimitPos * -1;
            // Used to save current position of stick
            var movedX = centerX;
            var movedY = centerY;
            
            
                
            // Check if the device support the touch or not
            if("ontouchstart" in document.documentElement)
            {
                
                canvas.addEventListener("touchstart", onTouchStart, true);
                canvas.addEventListener("touchmove", onTouchMove, true);
                document.addEventListener("touchend", onTouchEnd, true);
                /*
                canvas.addEventListener("dragstart", onDragStart, false);
                canvas.addEventListener("dragend", onDragEnd, false);
                canvas.addEventListener("touchcancel", onTouchCancel, false);
                */
            }
            else
            {
                canvas.addEventListener("mousedown", onMouseDown, true);
                canvas.addEventListener("mousemove", onMouseMove, true);
                document.addEventListener("mouseup", onMouseUp, true);
            }
            // Draw the object
            drawExternal();
            drawInternal();

            /**
             * @desc Draw the external circle used as reference position
             */
            function drawExternal()
            {
                context.beginPath();
                context.arc(centerX, centerY, externalRadius, 0, circumference, false);
                context.lineWidth = externalLineWidth;
                context.strokeStyle = externalStrokeColor;
                context.stroke();
            }

            /**
             * @desc Draw the internal stick in the current position the user have moved it
             */
            function drawInternal()
            {
                context.beginPath();
                if(movedX<internalRadius) { movedX=maxMoveStick; }
                if((movedX+internalRadius) > canvas.width) { movedX = canvas.width-(maxMoveStick); }
                if(movedY<internalRadius) { movedY=maxMoveStick; }
                if((movedY+internalRadius) > canvas.height) { movedY = canvas.height-(maxMoveStick); }
                context.arc(movedX, movedY, internalRadius, 0, circumference, false);
                // create radial gradient
                var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200);
                // Light color
                grd.addColorStop(0, internalFillColor);
                // Dark color
                grd.addColorStop(1, internalStrokeColor);
                context.fillStyle = grd;
                context.fill();
                context.lineWidth = internalLineWidth;
                context.strokeStyle = internalStrokeColor;
                context.stroke();
            }

            function postCoordinate()
            { 
                joyX = (100*((movedX - centerX)/maxMoveStick)).toFixed();
                joyY = ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed();

                send_data();
            }

            function releaseControl()
            { 
                joyX = 0;
                joyY = 0;
                
                send_data();
            }  
            
            /**
             * @desc Events for manage touch
             */
            function noTouch(event)
            {
                isPressing = 0;
                isMoving = 0;
                isRelease = 0;
            }

            function onTouchStart(event) 
            {
                isPressing = 1;
                isMoving = 0;
                isRelease = 0;
            }

            function onTouchMove(event)
            {
                // Prevent the browser from doing its default thing (scroll, zoom)
                event.preventDefault();
                if(isPressing === 1 && event.targetTouches[0].target === canvas)
                {
                    isMoving = 1;
                    isRelease = 0;
                    
                    movedX = event.targetTouches[0].pageX;
                    movedY = event.targetTouches[0].pageY;
                    // Manage offset
                    if(canvas.offsetParent.tagName.toUpperCase() === "BODY")
                    {
                        movedX -= canvas.offsetLeft;
                        movedY -= canvas.offsetTop;
                    }
                    else
                    {
                        movedX -= canvas.offsetParent.offsetLeft;
                        movedY -= canvas.offsetParent.offsetTop;
                    }
                    // Delete canvas
                    context.clearRect(0, 0, canvas.width, canvas.height);
                    // Redraw object
                    drawExternal();
                    drawInternal();
                    
                    postCoordinate();
                }
            } 

            function onTouchEnd(event) 
            {
                //event.preventDefault();
                isPressing = 0;
                isMoving = 0;
                isRelease = 1;
                
                // If required reset position store variable
                if(autoReturnToCenter)
                {
                    movedX = centerX;
                    movedY = centerY;
                }
                // Delete canvas
                context.clearRect(0, 0, canvas.width, canvas.height);
                // Redraw object
                drawExternal();
                drawInternal();
                //canvas.unbind('touchmove');
                
                releaseControl();
            }
    

            /**
             * @desc Events for manage mouse
             */
            function noMouse(event)
            {
                isPressing = 0;
                isMoving = 0;
                isRelease = 0;
            }

            function onMouseDown(event) 
            {
                isPressing = 1;
                isMoving = 0;
                isRelease = 0;
            }

            function onMouseMove(event) 
            {
                if(isPressing === 1)
                {
                    isMoving = 1;
                    isRelease = 0;
                    
                    movedX = event.pageX;
                    movedY = event.pageY;
                    // Manage offset
                    if(canvas.offsetParent.tagName.toUpperCase() === "BODY")
                    {
                        movedX -= canvas.offsetLeft;
                        movedY -= canvas.offsetTop;
                    }
                    else
                    {
                        movedX -= canvas.offsetParent.offsetLeft;
                        movedY -= canvas.offsetParent.offsetTop;
                    }
                    // Delete canvas
                    context.clearRect(0, 0, canvas.width, canvas.height);
                    // Redraw object
                    drawExternal();
                    drawInternal();
                    
                    postCoordinate();
                }
            }

            function onMouseUp(event) 
            {
                isPressing = 0;
                isMoving = 0;
                isRelease = 1;
                
                // If required reset position store variable
                if(autoReturnToCenter)
                {
                    movedX = centerX;
                    movedY = centerY;
                }
                // Delete canvas
                context.clearRect(0, 0, canvas.width, canvas.height);
                // Redraw object
                drawExternal();
                drawInternal();
                //canvas.unbind('mousemove');
            
                releaseControl();
            }


            this.GetX = function ()
            {
                return (100*((movedX - centerX)/maxMoveStick)).toFixed();
            };

            /**
             * @desc Normalizzed value of Y move of stick
             * @return Integer from -100 to +100
             */
            this.GetY = function ()
            {
                return ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed();
            };
            

        });


            var joy1Param = { "title": "1" };  
            var Joy1 = new JoyStick('joy1Div', joy1Param);
            var joy1X = document.getElementById("joy1X");
            var joy1Y = document.getElementById("joy1Y");
            setInterval(function()
            { 
                joyX = Joy1.GetX();
                joyY = Joy1.GetY();
                send_data();
            }, 150);


    </script> 
                
       
</body>
</html>



)=====";
