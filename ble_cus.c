#include "sdk_common.h"
#include "ble_cus.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "mem_manager.c"
#include "mem_manager.h"
 #include "sx127x_drv.c"
 
static uint8_t lora_txpkt[15]; /*LORA packet*/
static uint32_t sendcycle=0; /*LORA packet*/

static int combinedsize=0; //for viktor_value write
static int readnumber=0; //for cccd read
char sendbuffer[160]; 

void ble_lora_send(void){
	
	     if(sendcycle==10){
						sendcycle=0;
					}
					else{
						sendcycle++;
					}
			    	
			    	memcpy(lora_txpkt,(sendbuffer+(sendcycle*15)),15*sizeof(uint8_t));
					sx127x_sendPkt(lora_txpkt); 
}



/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_DISCONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}



/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 *//*Once we get the Write event we have to get hold of the Write event parameters that are passed with the event
 and we have to verify that the the handle that is written to matches the Custom Value Characteristic handle*/
static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;//get hold of the Write event parameters
    uint32_t i=0;
	
    // Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_cus->custom_value_handles.value_handle)
    {
        //nrf_gpio_pin_toggle(LED_4);
		
        NRF_LOG_INFO("In ble_cus_custom_value_update. \r\n"); 
        if(*p_evt_write->data == 0x01) //a dataez egy változó hosszúságú tömb
        {
            nrf_gpio_pin_clear(20); 
			
        }
        else if(*p_evt_write->data == 0x02)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
         //do nothing
        }
		
    }
	
	/**Viktor valuet irjuk**/
	 if (p_evt_write->handle == p_cus->viktor_value_handles.value_handle)
    {
	if(*p_evt_write->data == 0x01)
        {
            nrf_gpio_pin_clear(20); 
        }	
		else if(*p_evt_write->data == 0x02)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
         //do nothing
        }	
		uint32_t err_code = NRF_SUCCESS;
		
		//copying the just got data to this buffer "static char sendbuffer[160]"
		for(i=combinedsize;i<p_evt_write->len+combinedsize;i++)
		{
			sendbuffer[i]=p_evt_write->data[i-combinedsize];
		}
		combinedsize+=p_evt_write->len;
		
		
		//160 bytes can be sent to the other device over LORA
		if(160-combinedsize<20){
			combinedsize=0;
		}
		
		NRF_LOG_INFO("%d",combinedsize);
		/**csúnya megoldás**/
    }
	
	/**/

    // Check if the Viktor value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_cus->viktor_value_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {
		if(*p_evt_write->data == 0x0100)
        {
            nrf_gpio_pin_clear(20); 
        }	
		else if(*p_evt_write->data == 0x0000)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
         //do nothing
        }	
        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {
            ble_cus_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }

}
//Lekezeli a ble_stack eventeket.Mikor hívódik meg? 
//!Ha valami történik a Softdevice-on. Regisztrálva van, hogy figyel.

void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_t * p_cus = (ble_cus_t *) p_context;
    
    NRF_LOG_INFO("BLE event received. Event type = %d\r\n", p_ble_evt->header.evt_id); 
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cus, p_ble_evt);
            break;
/*Whenever a characteristic is written to, a BLE_GATTS_EVT_WRITE event will be propagated to the application
? and dispatched to the functions in ble_evt_dispatch()? ez vajon mit jelent*/
        case BLE_GATTS_EVT_WRITE:
		NRF_LOG_INFO("Valami\n\r");
		NRF_LOG_FLUSH();
            on_write(p_cus, p_ble_evt);
            break;
/* Handling this event is not necessary
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.\r\n");
            break;
*/
        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
 /*A service inicializáló fóggvényben keról meghívásra.
 Első paraméterként megkapja a serviceünket,másodikként pedig az inicializáláshoz szükséges adatokat
 This function will add both the Characteristic Declaration and the Characteristic Value Declaration to our attribute table*/
static uint32_t custom_value_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md; //Characteristic Declaration: a tulajdonságokat állítjuk be vele amit a discovery során közlünk a centrallal
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value; //Characteristic Value Declaration: ezt kell átadni a karakterisztikát létrehozú függvénynek
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;  //itt állítjuk be ténylegesen a tulajdonságokat(attr_char_value-nak kell odaadni)
								  //engedélyeket és felhatalmazásokat tárol+hol van a characteristic value+változó hosszúságú-e

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    /*The descriptor is an attribute with additional information about the characteristic.
	The CCCD is a writable descriptor that allows the client, 
	i.e. your MCP or phone, to enable or disable notification or indication, on your kit.*/
    cccd_md.write_perm = p_cus_init->viktor_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;//NEM akarom engedélyezni az írását
    char_md.char_props.notify = 0;  //Disable notification by setting the notify property bit to 0
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL; //&cccd_md volt; 
    char_md.p_sccd_md         = NULL;
	/*This will add a Client Characteristic Configuration Descriptor or CCCD to the Custom Value Characteristic
	which allows us to enable or disable notifications by writing to the CCCD.
	Notification is by default disabled and in order to enable it we have to write 0x0001 to the CCCD. 
	Remember that everytime we write to a characteristic or one of its descriptors, we will get a Write event, 
	thus we need to handle the case where a peer writes to the CCCD in the on_write() function.*/
		
    ble_uuid.type = p_cus->uuid_type;		//beállítjuk a karakterisztika UUID-jét és annak a típusát is
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID; //pointerként továbbadjuk a karakterisztika értékének

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;  //tényleges engedélyezés
   //NE lehessen írni// attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm; //ez is
    attr_md.vloc       = BLE_GATTS_VLOC_STACK; //stackbe tároljuk a metadatát
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
	
   char value[]            = "1:V1 segelykero allomas GPS:47.470 19.0566"; // inicializáljuk

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = strlen(value); // eredetileg 'sizeof(uint8_t);' ez 1 byte
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = strlen(value); // eredetileg 'sizeof(uint8_t);'
	
	attr_char_value.p_value     = value;
/*1:melyik servicehez tartozik,2:a leíró metadata(magyarul mit tud),3:mi az 4:ide kapjuk meg a handlejét a krakterisztiknak*/
    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, 
												&char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	/*return NRF_SUCCESS;*/
	
	/** Viktor characteristicát inicializáljuk **/

    ble_uuid.type = p_cus->uuid_type;		//beállítjuk a karakterisztika UUID-jét és annak a típusát is
    ble_uuid.uuid = VIKTOR_VALUE_CHAR_UUID; //pointerként továbbadjuk a karakterisztika értékének
	attr_char_value.p_uuid    = &ble_uuid;
	
	char_md.char_props.notify = 1;  //Enable notification by setting the notify property 
	char_md.p_cccd_md         = &cccd_md;
	char_md.char_props.write  = 1; //őt lehessen írni
	
	attr_md.read_perm  = p_cus_init->viktor_value_char_attr_md.read_perm;  //tényleges engedélyezés
    attr_md.write_perm = p_cus_init->viktor_value_char_attr_md.write_perm; //ez is
	
	uint8_t viktor_value_init[1]={0};
	attr_char_value.init_len  = sizeof(uint8_t); // It has 1 byte at the beggining
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20*sizeof(uint8_t); 	   // eredetileg 'sizeof(uint8_t);
	attr_char_value.p_value     = viktor_value_init;

/*1:melyik servicehez tartozik,2:a leíró metadata(magyarul mit tud),3:mi az 4:ide kapjuk meg a handlejét a krakterisztikának*/
    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, 
												&char_md,
                                               &attr_char_value,
                                               &p_cus->viktor_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	return NRF_SUCCESS;
	
	
	
}

/*Az első paraméter maga a service lesz amit feltöltünk adatokkal és később arra lesz használva hogy azonosítsuk az adott service-t
	A második paraméter: a service inicializáláshoz szükséges extra adatokat tartalmaz*/
uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID; //2 sor a service struktúránk inicializálásának kezdete

    // Add Custom Service UUID 
	// A vendor specific UUID-nket hozzáadjuk a BLE stackben lévő UUID táblához
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);
    /* az előző függvényben visszakaptunk egy értéket(paraméterben) 
	és ezt most az összes ble_uuid_t típusú változónknak be kell állítanunk,
	így ezek után a saját BASE_UUID-nket fogjuk használni*/
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID; //ezt mi állítjuk be tetszőlegesen amire akarjuk(az egész BASE UUID-t látjuk az App-ban, de abból a második 16 bit lesz az általunk beállított UUID(itt 1400))

    // Add the Custom Service
	/*Létrehoz egy táblát ami a mi szervizeinket tárolja és a service_handle az éppen aktuális service helyére mutat
	Paraméterek 1:Milyen servicet akarunk 
				  2:pointer a service UUID-nkre amit most csináltunk(egyedileg azonosítva lesz a BLE STACK-ben
				  3:A service-handle számot hova tegye(saját service service_handler mezőjébe)*/
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	//Add Viktor Value Charateristic
	/*viktor_value_char_add(p_cus, p_cus_init);*/
	
    // Add Custom Value characteristic
    return custom_value_char_add(p_cus, p_cus_init);
}
uint32_t lora_value_update(ble_cus_t * p_cus, uint8_t *pktData)
{
	   NRF_LOG_INFO("In ble_cus_custom_value_update. \r\n"); 
	int i;
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
	}
	uint32_t err_code = NRF_SUCCESS;
	uint8_t value[20];
	
	for(i=0;i<20;i++)  {
		value[i]=pktData[(20*readnumber)+i];
		}
	if(readnumber==7)
	{ 
		readnumber=0;
	} 
	else  {
		readnumber++;
		  }
		
    ble_gatts_value_t gatts_value;
	
    // Initialize value struct; NOW:Update the value in the GATT table
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 20; 
    gatts_value.offset  = 0;
    gatts_value.p_value = value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->viktor_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	/*After updating the value in the GATT table we're ready to notify our peer 
	that the value of our Custom Value Characteristic has changed.*/
    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
	/*hvx stands for Handle Value X, where X symbolize either notification or indication 
	as the struct and function can be used for both*/
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->viktor_value_handles.value_handle;//The SoftDevice needs to know what characteristic value we are working on
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;//The other option would be BLE_GATT_HVX_INDICATION.
        hvx_params.offset = gatts_value.offset;//Your characteristic value might be a sequence of many bytes. If you want to transmit only a couple of these bytes and the bytes are located in the middle of the sequence you can use the offset to extract them.
        hvx_params.p_len  = &gatts_value.len;//The SoftDevice needs to know how many bytes to transmit. 
        hvx_params.p_data = gatts_value.p_value; /*eredetileg valami random memóriatartalmat inkrementáltunk*/

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }


    return err_code;
	
}

/*This is where we will implement the notification of !viktor_value!*/
uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t custom_value)
{
    NRF_LOG_INFO("In ble_cus_custom_value_update. \r\n"); 
	int i;
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
	}
	uint32_t err_code = NRF_SUCCESS;
	uint8_t value[20];
	
	for(i=0;i<20;i++)  {
		value[i]=sendbuffer[(20*readnumber)+i];
		}
	if(readnumber==7)
	{ 
		readnumber=0;
	} 
	else  {
		readnumber++;
		  }
		
    ble_gatts_value_t gatts_value;
	
    // Initialize value struct; NOW:Update the value in the GATT table
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 20; 
    gatts_value.offset  = 0;
    gatts_value.p_value = value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->viktor_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	/*After updating the value in the GATT table we're ready to notify our peer 
	that the value of our Custom Value Characteristic has changed.*/
    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
	/*hvx stands for Handle Value X, where X symbolize either notification or indication 
	as the struct and function can be used for both*/
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->viktor_value_handles.value_handle;//The SoftDevice needs to know what characteristic value we are working on
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;//The other option would be BLE_GATT_HVX_INDICATION.
        hvx_params.offset = gatts_value.offset;//Your characteristic value might be a sequence of many bytes. If you want to transmit only a couple of these bytes and the bytes are located in the middle of the sequence you can use the offset to extract them.
        hvx_params.p_len  = &gatts_value.len;//The SoftDevice needs to know how many bytes to transmit. 
        hvx_params.p_data = gatts_value.p_value; /*eredetileg valami random memóriatartalmat inkrementáltunk*/

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }


    return err_code;
}
