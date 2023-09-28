/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_RAW && LWIP_NETCONN && LWIP_DHCP && LWIP_DNS

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_phy.h"

#include "lwip/api.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dhcp.h"
#include "lwip/netdb.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip_mqtt_id.h"

#include "ctype.h"
#include "stdio.h"

#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"
#include "fsl_device_registers.h"
#include "event_groups.h"
#include <time.h> /* Function for  srand */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @TEST_ANCHOR */

/* MAC address configuration. */
#ifndef configMAC_ADDR
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x14 \
    }
#endif

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/* GPIO pin configuration. */
#define BOARD_LED_R_GPIO 		BOARD_LED_RED_GPIO
#define BOARD_LED_R_GPIO_PIN   	BOARD_LED_RED_GPIO_PIN
#define BOARD_LED_G_GPIO       	BOARD_LED_GREEN_GPIO
#define BOARD_LED_G_GPIO_PIN   	BOARD_LED_GREEN_GPIO_PIN
#define BOARD_LED_B_GPIO       	BOARD_LED_BLUE_GPIO
#define BOARD_LED_B_GPIO_PIN   	BOARD_LED_BLUE_GPIO_PIN

/*Event Bits*/
#define HUM_TEMP_QUANT_EVT		( 1 << 0 )
#define SOUND_EVT				( 1 << 1 )
#define SAMPLE_RATE_EVT			( 1 << 2 )
#define MIC_SENSITIVITY			( 1 << 3 )
#define SLEEP_EVT				( 1 << 4 )

#define HUMIDITY_RANGE			100u /**Porcentage*/
#define HONEY_QUANTITY_RANGE	500u /**Grams*/
#define TEMP_RANGE				100 /**Celcius*/


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*! @brief MQTT server host name or IP address. */
#define EXAMPLE_MQTT_SERVER_HOST "driver.cloudmqtt.com"

/*! @brief MQTT server port number. */
#define EXAMPLE_MQTT_SERVER_PORT 18591

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*! @brief Stack size of the temporary initialization thread. */
#define APP_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary initialization thread. */
#define APP_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void connect_to_mqtt(void *ctx);
void xtimer_sensor_callback(TimerHandle_t pxTimer);
void xtimer_mic_callback(TimerHandle_t pxTimer);
void generate_random_values(uint32_t sensitivity);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

/*! @brief MQTT client data. */
static mqtt_client_t *mqtt_client;

/*! @brief MQTT client ID string. */
static char client_id[40];

/*! @brief MQTT client information. */
static const struct mqtt_connect_client_info_t mqtt_client_info = {
    .client_id   = (const char *)&client_id[0],
    .client_user = "omar_o2023",
    .client_pass = "o2023",
    .keep_alive  = 100,
    .will_topic  = NULL,
    .will_msg    = NULL,
    .will_qos    = 0,
    .will_retain = 0,
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    .tls_config = NULL,
#endif
};

/*! @brief MQTT broker IP address. */
static ip_addr_t mqtt_addr;

/*! @brief Indicates connection to MQTT broker. */
static volatile bool connected = false;

/*For LED config*/
gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};

/*Holds the even*/
EventGroupHandle_t 	xEventGroup;
TimerHandle_t 		xTimer_Sensor;
TimerHandle_t 		xTimer_Mic;

/**Global flags for application*/
uint8_t 	g_ONOFF_flag = 0;
uint32_t 	g_Mic_sensitivity_range;
uint32_t 	g_Sample_rate;

uint16_t g_humid_val;
uint16_t g_temp_val;
uint16_t g_honeyq_val;
uint16_t g_sound_val;
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Called when subscription request finishes.
 */
static void mqtt_topic_subscribed_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Subscribed to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to subscribe to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Called when there is a message on a subscribed topic.
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG(arg);

    PRINTF("Received %u bytes from the topic \"%s\": \"", tot_len, topic);
}

/*!
 * @brief Called when recieved incoming published message fragment.
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    int i;

    LWIP_UNUSED_ARG(arg);

    for (i = 0; i < len; i++)
    {
        if (isprint(data[i]))
        {
            PRINTF("%c", (char)data[i]);
        }
        else
        {
            PRINTF("\\x%02x", data[i]);
        }
    }

    if (flags & MQTT_DATA_FLAG_LAST)
    {
        PRINTF("\"\r\n");
    }
}

/*!
 * @brief Subscribe to MQTT topics.
 */
static void mqtt_subscribe_topics(mqtt_client_t *client)
{
    static const char *topics[] = {"hive/1/humidity", "hive/1/temperature", "hive/1/honey_qt",
    							   "hive/1/sound", "hive/1/sample_rate", "hive/1/mic_sensitivity"};


    int qos[]                   = {0, 0, 0, 0, 0, 0};
    err_t err;
    int i;

    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            LWIP_CONST_CAST(void *, &mqtt_client_info));

    for (i = 0; i < ARRAY_SIZE(topics); i++)
    {
        err = mqtt_subscribe(client, topics[i], qos[i], mqtt_topic_subscribed_cb, LWIP_CONST_CAST(void *, topics[i]));

        if (err == ERR_OK)
        {
            PRINTF("Subscribing to the topic \"%s\" with QoS %d...\r\n", topics[i], qos[i]);
        }
        else
        {
            PRINTF("Failed to subscribe to the topic \"%s\" with QoS %d: %d.\r\n", topics[i], qos[i], err);
        }
    }
}

/*!
 * @brief Called when connection state changes.
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

    connected = (status == MQTT_CONNECT_ACCEPTED);

    switch (status)
    {
        case MQTT_CONNECT_ACCEPTED:
            PRINTF("MQTT client \"%s\" connected.\r\n", client_info->client_id);
            mqtt_subscribe_topics(client);
            break;

        case MQTT_CONNECT_DISCONNECTED:
            PRINTF("MQTT client \"%s\" not connected.\r\n", client_info->client_id);
            /* Try to reconnect 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_TIMEOUT:
            PRINTF("MQTT client \"%s\" connection timeout.\r\n", client_info->client_id);
            /* Try again 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
        case MQTT_CONNECT_REFUSED_SERVER:
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            PRINTF("MQTT client \"%s\" connection refused: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;

        default:
            PRINTF("MQTT client \"%s\" connection status: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;
    }
}

/*!
 * @brief Starts connecting to MQTT broker. To be called on tcpip_thread.
 */
static void connect_to_mqtt(void *ctx)
{
    LWIP_UNUSED_ARG(ctx);

    PRINTF("Connecting to MQTT broker at %s...\r\n", ipaddr_ntoa(&mqtt_addr));

    mqtt_client_connect(mqtt_client, &mqtt_addr, EXAMPLE_MQTT_SERVER_PORT, mqtt_connection_cb,
                        LWIP_CONST_CAST(void *, &mqtt_client_info), &mqtt_client_info);
}

/*!
 * @brief Called when publish request finishes.
 */
static void mqtt_message_published_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Published to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to publish to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Publishes a message. To be called on tcpip_thread.
 */
static void publish_message(void *ctx)
{
    static const char *topic   = "omar_o2023/100";
    static const char *message = "message from Omar";

    LWIP_UNUSED_ARG(ctx);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);

    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
}

/*!
 * @brief Application thread.
 */
static void app_thread(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    err_t err;
    int i;
    /**My variables for MQTT application*/
    TickType_t xTicksToWait = 1000 / portTICK_PERIOD_MS;
    EventBits_t uxBits;
    xEventGroup = xEventGroupCreate();
    if(xEventGroup == NULL)
    {
    	PRINTF("Error Creating Event Group\r\n");
    }
    /*Create Timer*/
    xTimer_Sensor = xTimerCreate("TimerSensor", 5000 / portTICK_PERIOD_MS, pdTRUE, (void*)0, xtimer_sensor_callback);
    if(xTimer_Sensor == NULL)
    {
    	PRINTF("Error Creating Timer\r\n");
    }
    xTimer_Mic = xTimerCreate("TimerMic", 7000 / portTICK_PERIOD_MS, pdTRUE, (void*)4, xtimer_mic_callback);


    /* Wait for address from DHCP */

    PRINTF("Getting IP address from DHCP...\r\n");

    do
    {
        if (netif_is_up(netif))
        {
            dhcp = netif_dhcp_data(netif);
        }
        else
        {
            dhcp = NULL;
        }

        sys_msleep(20U);

    } while ((dhcp == NULL) || (dhcp->state != DHCP_STATE_BOUND));

    PRINTF("\r\nIPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
    PRINTF("IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
    PRINTF("IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));

    /*
     * Check if we have an IP address or host name string configured.
     * Could just call netconn_gethostbyname() on both IP address or host name,
     * but we want to print some info if goint to resolve it.
     */
    if (ipaddr_aton(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr) && IP_IS_V4(&mqtt_addr))
    {
        /* Already an IP address */
        err = ERR_OK;
    }
    else
    {
        /* Resolve MQTT broker's host name to an IP address */
        PRINTF("Resolving \"%s\"...\r\n", EXAMPLE_MQTT_SERVER_HOST);
        err = netconn_gethostbyname(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr);
    }

    if (err == ERR_OK)
    {
        /* Start connecting to MQTT broker from tcpip_thread */
        err = tcpip_callback(connect_to_mqtt, NULL);
        if (err != ERR_OK)
        {
            PRINTF("Failed to invoke broker connection on the tcpip_thread: %d.\r\n", err);
        }
    }
    else
    {
        PRINTF("Failed to obtain IP address: %d.\r\n", err);
    }

    /* Publish some messages */
    while (1)
    {
        if (connected)
        {
        	xTimerStart(xTimer_Sensor,0);
        	uxBits = xEventGroupWaitBits(xEventGroup,
        								HUM_TEMP_QUANT_EVT | SOUND_EVT | SAMPLE_RATE_EVT | MIC_SENSITIVITY | SLEEP_EVT,
										pdTRUE,
										pdFALSE,
										xTicksToWait);

        	if(uxBits == 0) continue; /*Returns to while*/

        	if(uxBits & HUM_TEMP_QUANT_EVT)
        	{
        		/**Publish Temperature,Humidity and Honey Quantity*/
        	}
        	else if(uxBits & SOUND_EVT)
        	{
        		/**Publish Sound from rand function*/
        	}
        	else if(uxBits & SAMPLE_RATE_EVT)
        	{
        		/**Modifies sample rate simulated with Timer
        		 * Use xTimerChangePeriodFromISR() */
        	}
        	else if(uxBits & MIC_SENSITIVITY)
        	{
        		/**Modifies Mic Sensitivity, increase the range of random number*/
        	}
        	else if(uxBits & SLEEP_EVT)
        	{
        		/**Send device to sleep, Turns ON LED
        		 * LED ON: Device is on Sleep
        		 * LED OFF: DEvice is working normal*/
        		if(g_ONOFF_flag == 0)
        		{
        			GPIO_PortSet(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN); /**Turns OFF LED*/
        		}
        		else
        		{
        			GPIO_PortClear(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN); /**Turns ON, Sleep mode*/
        		}
        	}
        	else
        	{
        		PRINTF("Unknown Event");
        	}


//            err = tcpip_callback(publish_message, NULL);
//            if (err != ERR_OK)
//            {
//                PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
//            }
        }
    }
    vTaskDelete(NULL);
}

static void generate_client_id(void)
{
    uint32_t mqtt_id[MQTT_ID_SIZE];
    int res;

    get_mqtt_id(&mqtt_id[0]);

    res = snprintf(client_id, sizeof(client_id), "nxp_%08lx%08lx%08lx%08lx", mqtt_id[3], mqtt_id[2], mqtt_id[1],
                   mqtt_id[0]);
    if ((res < 0) || (res >= sizeof(client_id)))
    {
        PRINTF("snprintf failed: %d\r\n", res);
        while (1)
        {
        }
    }
}

/*!
 * @brief Initializes lwIP stack.
 *
 * @param arg unused
 */
static void stack_init(void *arg)
{
    static struct netif netif;
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
    };

    LWIP_UNUSED_ARG(arg);
    generate_client_id();

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR(&netif_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_gw, 0U, 0U, 0U, 0U);

    tcpip_init(NULL, NULL);

    LOCK_TCPIP_CORE();
    mqtt_client = mqtt_client_new();
    UNLOCK_TCPIP_CORE();
    if (mqtt_client == NULL)
    {
        PRINTF("mqtt_client_new() failed.\r\n");
        while (1)
        {
        }
    }

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    netifapi_dhcp_start(&netif);

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" MQTT client example\r\n");
    PRINTF("************************************************\r\n");

    if (sys_thread_new("app_task", app_thread, &netif, APP_THREAD_STACKSIZE, APP_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("stack_init(): Task creation failed.", 0);
    }

    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    SYSMPU_Type *base = SYSMPU;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;

    /* Initialize lwIP from thread */
    if (sys_thread_new("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}

void generate_random_values(uint32_t sensitivity)
{
	srand(time(NULL)); /*Generates random number*/
	g_humid_val = rand() % (HUMIDITY_RANGE + 1);

	srand(time(NULL)); /*Generates random number*/
	g_temp_val = rand() % (TEMP_RANGE + 1);

	srand(time(NULL)); /*Generates random number*/
	g_honeyq_val = rand() % (HONEY_QUANTITY_RANGE + 1);

	srand(time(NULL)); /*Generates random number*/
	g_sound_val =rand() % (sensitivity + 1);
}

/**Functions to publish*/
void publish_hum(void *ctx)
{
    static const char *topic   = "omar_o2023/100";//TODO
    static char message[16];

    LWIP_UNUSED_ARG(ctx);
    memset(message,0,16);
    sprintf(message,"%d",g_humid_val);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
}

void publish_temp(void *ctx)
{
    static const char *topic   = "omar_o2023/100";//TODO
    static char message[16];

    LWIP_UNUSED_ARG(ctx);
    memset(message,0,16);
    sprintf(message,"%d",g_temp_val);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
}

void publish_honey(void *ctx)
{
    static const char *topic   = "omar_o2023/100";//TODO
    static char message[16];

    LWIP_UNUSED_ARG(ctx);
    memset(message,0,16);
    sprintf(message,"%d",g_honeyq_val);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
}

void publish_sound(void *ctx)
{
    static const char *topic   = "omar_o2023/100";//TODO
    static char message[16];

    LWIP_UNUSED_ARG(ctx);
    memset(message,0,16);
    sprintf(message,"%d",g_sound_val);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);
    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
}

void xtimer_sensor_callback(TimerHandle_t pxTimer)
{
	xEventGroupSetBits(xEventGroup, HUM_TEMP_QUANT_EVT);
}

void xtimer_mic_callback(TimerHandle_t pxTimer)
{
	xEventGroupSetBits(xEventGroup, SOUND_EVT);
}
#endif
