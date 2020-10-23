1.请分别在以下目录打上附件补丁：
  framework/base
  framework/native
  pacakge/app/Settings
2.打开Setting应用，在Display->HDMI界面中分别通过Second screen full display，Second screen rotation选项来调整副屏是否
  满屏显示以及副屏的方向。
3.方向以及显示调整完，可以通过getprop persist.sys.rotation.efull, getprop persist.sys.rotation.einit分别查看这两个属性值，并
  在系统中加入该属性，开机之后即可达到效果。