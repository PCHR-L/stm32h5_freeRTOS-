#include "ui.h"

lv_obj_t *led = NULL;
lv_obj_t *led1 = NULL;
lv_obj_t *led2 = NULL;

lv_obj_t *lab_temp = NULL;
lv_obj_t *lab_volt = NULL;
lv_obj_t *env_light = NULL;
lv_obj_t *env_press = NULL;

/**
 * @brief Initialize the UI
 *
 */
void ui_init(void)
{
    //自定义页面设备
    //1.获取当前硬件整个屏幕
    lv_obj_t *src = lv_screen_active();
    
    //3.创建部件
        //图片
    LV_IMAGE_DECLARE(m1y);
    lv_obj_t * image = lv_image_create(src);
    lv_image_set_src(image, &m1y);
    lv_obj_align(image, LV_ALIGN_CENTER, 130,-50);

    //字体
    LV_FONT_DECLARE(font30);
    lv_obj_t * label = lv_label_create(src);
    lv_obj_t * label1 = lv_label_create(src);
    lv_obj_set_style_text_font(label, &font30, 0);
    lv_obj_set_style_text_font(label1, &font30, 0);
    lv_label_set_text(label, "工业物联网设备");
    lv_label_set_text(label1, "管理系统");
    lv_obj_set_pos(label, 30, 10);
    lv_obj_set_pos(label1, 75, 40);

    //开关量传感器
    lv_obj_t * switch_label = lv_label_create(src);
    lv_label_set_text(switch_label, "switch sensor:");
    lv_obj_set_pos(switch_label, 10, 100);
    //按键响应1
    led = lv_led_create(src);
    lv_led_set_color(led, lv_color_hex(0x00ff00));
    lv_obj_set_pos(led, 20, 125);
    lv_led_off(led);
    //按键响应2
    led1 = lv_led_create(src);
    lv_led_set_color(led1, lv_color_hex(0xff0000));
    lv_obj_set_pos(led1, 60, 125);
    lv_led_off(led1);
    //按键响应3
    led2 = lv_led_create(src);
    lv_led_set_color(led2, lv_color_hex(0x0000ff));
    lv_obj_set_pos(led2, 100, 125);
    lv_led_off(led2);
    
    //温湿度传感器
    lv_obj_t * tah_label = lv_label_create(src);
    lv_label_set_text(tah_label, "temp&humi sensor:");
    lv_obj_set_pos(tah_label, 10, 160);
    
    lab_temp = lv_label_create(src);
    lv_obj_align(lab_temp, LV_ALIGN_TOP_LEFT, 20, 180);
    lab_volt = lv_label_create(src);
    lv_obj_align(lab_volt, LV_ALIGN_TOP_LEFT, 20, 200);

    //环境检测
    lv_obj_t * env_label = lv_label_create(src);
    lv_label_set_text(env_label, "env sensor:");
    lv_obj_set_pos(env_label, 10, 220);
    
    env_light = lv_label_create(src);
    lv_obj_align(env_light, LV_ALIGN_TOP_LEFT, 20, 240);
    env_press = lv_label_create(src);
    lv_obj_align(env_press, LV_ALIGN_TOP_LEFT, 20, 260);
}
