#include "grabbing.h"

void grabbing_init(void)
{
    //mcp2515_sys_init(&grabbing_can,&hspi3,GPIOA,GPIO_PIN_15);
	//mcp2515_sys_init(&grabbing_can,&hspi2,GPIOB,GPIO_PIN_12);
	mcp2515_sys_init(&grabbing_can,&hspi1,GPIOA,GPIO_PIN_4);
}

void grabbing_enable(void)
{
    dm_enable_mcp2515(&grabbing_can, 0x01);//閸楀洭妾?
    vTaskDelay(1);
    dm_enable_mcp2515(&grabbing_can, 0x02);//闂堢姳绗呴惃鍕仚閻栴亞娈憄itch
    vTaskDelay(1);
    dm_enable_mcp2515(&grabbing_can, 0x03);//闂堢姳绗呴惃鍕仚閻栴亞娈戞导鍝ョ級
    vTaskDelay(1);
    dm_enable_mcp2515(&grabbing_can, 0x04);//闂堢姳绗傞惃鍕仚閻栴亞娈憄itch
    vTaskDelay(1);
    dm_enable_mcp2515(&grabbing_can, 0x05);//闂堢姳绗傞惃鍕仚閻栴亞娈戞导鍝ョ級
	vTaskDelay(1);
}

void grabbing_disable(void)
{
    dm_disable_mcp2515(&grabbing_can, 0x01);
    vTaskDelay(1);
    dm_disable_mcp2515(&grabbing_can, 0x02);
    vTaskDelay(1);
    dm_disable_mcp2515(&grabbing_can, 0x03);
    vTaskDelay(1);
    dm_disable_mcp2515(&grabbing_can, 0x04);
    vTaskDelay(1);
    dm_disable_mcp2515(&grabbing_can, 0x05);
	vTaskDelay(1);
}

//閸斻劋缍?
//婢跺湱鍩呴敍鍫滅瑓閿?1.85閺勵垰銇欓崣鏍︾秴缂冾噯绱濋崡鍥60閺勵垯绗呮径鐟板絿妤傛ê瀹虫禒銉ュ挤閸滃2閻ㄥ嫭顒熼崳銊ф畱鐎佃甯存妯哄
//娴煎摜缂夐敍鍫滅瑓閿?閺勵垱婀导绋垮毉閿?5閺勵垯鍑犻崙?
void grabbing_section1(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x03, 0.0f, 5.0f);
}

void grabbing_section2(void)
{
    YV2(1);
}

void grabbing_section3(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x03, 0.0f, 5.0f);
    pos_ctrl_mcp2515(&grabbing_can, 0x02, -1.85f, 10.0f);
}

void grabbing_section4(void)
{
    YV2(0);
}

void grabbing_section5(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x02, 0.0f, 20.0f);
}

void grabbing_section6(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x03, 10.0f, 5.0f);
}

void grabbing_upper_section1(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x05, 0.0f, 5.0f);
}

void grabbing_upper_section2(void)
{
    YV3(1);
}

void grabbing_upper_section3(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x05, 0.0f, 5.0f);
    pos_ctrl_mcp2515(&grabbing_can, 0x04, 1.85f, 10.0f);
}

void grabbing_upper_section4(void)
{
    YV3(0);
}

void grabbing_upper_section5(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x04, 0.0f, 20.0f);
}

void grabbing_upper_section6(void)
{
    pos_ctrl_mcp2515(&grabbing_can, 0x05, -10.0f, 5.0f);
}

void grabbing_push_to(uint8_t station, float target)
{
    if(station == GRABBING_STATION_UPPER)
        pos_ctrl_mcp2515(&grabbing_can, 0x05, -target, 5.0f);
    else
        pos_ctrl_mcp2515(&grabbing_can, 0x03, target, 5.0f);
}

void grabbing_run_step(uint8_t station, uint8_t step)
{
    if(station == GRABBING_STATION_UPPER)
    {
        switch(step)
        {
            case GRABBING_STEP_RETRACT:
                grabbing_upper_section1();
                break;

            case GRABBING_STEP_CLAW_OPEN:
                grabbing_upper_section2();
                break;

            case GRABBING_STEP_FLIP_DOWN:
                grabbing_upper_section3();
                break;

            case GRABBING_STEP_CLAW_CLOSE:
                grabbing_upper_section4();
                break;

            case GRABBING_STEP_FLIP_BACK:
                grabbing_upper_section5();
                break;

            case GRABBING_STEP_PUSH:
                grabbing_upper_section6();
                break;

            case GRABBING_STEP_PUSH_INCREMENT:
                break;

            default:
                break;
        }
    }
    else
    {
        switch(step)
        {
            case GRABBING_STEP_RETRACT:
                grabbing_section1();
                break;

            case GRABBING_STEP_CLAW_OPEN:
                grabbing_section2();
                break;

            case GRABBING_STEP_FLIP_DOWN:
                grabbing_section3();
                break;

            case GRABBING_STEP_CLAW_CLOSE:
                grabbing_section4();
                break;

            case GRABBING_STEP_FLIP_BACK:
                grabbing_section5();
                break;

            case GRABBING_STEP_PUSH:
                grabbing_section6();
                break;

            case GRABBING_STEP_PUSH_INCREMENT:
                break;

            default:
                break;
        }
    }
}
