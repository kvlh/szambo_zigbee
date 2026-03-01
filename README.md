# Szambo TOF Sensor — ESP32-C6 + VL53L0X + Zigbee + OTA

Czujnik poziomu szamba (zbiornik bezodpływowy) oparty na module XIAO ESP32-C6
z czujnikiem odległości VL53L0X (Time-of-Flight). Firmware napisany natywnie
w ESP-IDF z ESP-Zigbee-SDK. Urządzenie raportuje odległość, poziom napełnienia
i napięcie baterii do Home Assistant przez Zigbee2MQTT.

---

## Spis treści

1. [Sprzęt](#1-sprzęt)
2. [Architektura firmware](#2-architektura-firmware)
3. [Endpointy Zigbee i jednostki HA](#3-endpointy-zigbee-i-jednostki-ha)
4. [Zarządzanie energią — deep sleep](#4-zarządzanie-energią--deep-sleep)
5. [OTA — aktualizacja firmware przez sieć](#5-ota--aktualizacja-firmware-przez-sieć)
6. [Budowanie i wgrywanie firmware](#6-budowanie-i-wgrywanie-firmware)
7. [Konfiguracja Zigbee2MQTT](#7-konfiguracja-zigbee2mqtt)
8. [Konfiguracja w Home Assistant](#8-konfiguracja-w-home-assistant)
9. [NVS — trwałe ustawienia](#9-nvs--trwałe-ustawienia)
10. [Układ partycji flash](#10-układ-partycji-flash)
11. [Historia problemów i rozwiązań](#11-historia-problemów-i-rozwiązań)
12. [Schematy pinów](#12-schematy-pinów)
13. [Wersje oprogramowania](#13-wersje-oprogramowania)

---

## 1. Sprzęt

| Komponent | Model | Uwagi |
|---|---|---|
| Mikrokontroler | Seeed XIAO ESP32-C6 | Wbudowana antena PCB Zigbee 802.15.4 |
| Czujnik odległości | STMicroelectronics VL53L0X | ToF, zasięg 20–2000 mm |
| Zasilanie | LiPo 550 mAh lub 3.7V AA | Pomiar napięcia przez dzielnik na GPIO2 |

### Połączenia

| VL53L0X | ESP32-C6 (XIAO) |
|---|---|
| SDA | GPIO16 |
| SCL | GPIO17 |
| XSHUT | GPIO23 (SHUT) — sterowanie zasilaniem |
| VIN | 3.3V |
| GND | GND |

**Dzielnik napięcia baterii:**
- 200 kΩ na płytce XIAO (rezystor wbudowany)
- 100 kΩ dolutowany do masy
- Pomiar na GPIO2 (ADC1_CH2)
- Mnożnik kalibracyjny: `2.86` (wynikowy zakres 3.0–4.2 V → 0–100%)

---

## 2. Architektura firmware

```
app_main()
  ├── nvs_flash_init()
  ├── vl53l0x_init()       -- inicjalizacja I2C + pinu SHUT
  ├── esp_zb_platform_config()
  └── xTaskCreate(zigbee_task)

zigbee_task()
  ├── nvs_load_interval()  -- wczytaj interwał z NVS
  ├── nvs_load_tank_height()
  ├── esp_zb_init()        -- endpoint device type: END_DEVICE
  ├── Rejestracja EP1–EP5
  ├── esp_zb_core_action_handler_register()
  └── esp_zb_stack_main_loop()  -- blokuje; wywołuje callbacki

esp_zb_app_signal_handler()
  ├── SKIP_STARTUP  → start_top_level_commissioning(INIT)
  ├── DEVICE_FIRST_START / DEVICE_REBOOT
  │     ├── factory_new → network_steering
  │     └── commissioned → sensor_task_start()
  └── STEERING
        ├── ok → sensor_task_start()
        └── fail → retry steering

sensor_task_fn()  (oddzielny task FreeRTOS, jednorazowy)
  ├── esp_zb_ota_upgrade_client_query_interval_set(1)
  ├── adc_init()
  ├── vl53l0x_power_on() + read + power_off()
  ├── read_battery_voltage()
  ├── oblicz fill_level z tank_height
  ├── zigbee_report_value(EP_DISTANCE, distance_mm)
  ├── zigbee_report_value(EP_FILL_LEVEL, fill_pct)
  ├── zigbee_report_value(EP_BATTERY, battery_v)
  ├── vTaskDelay(3 s)   -- okno dla komend Z2M
  ├── pętla OTA_CHECK (90 s) -- czeka na OTA start
  ├── jeśli OTA → pętla do 90 min (lub do końca OTA)
  └── esp_deep_sleep_start(interval - elapsed)
```

### Pliki źródłowe

| Plik | Opis |
|---|---|
| `main/main.c` | Punkt wejścia: NVS, VL53L0X init, Zigbee platform config, start tasku |
| `main/zigbee_device.c` | Definicja endpointów, callbacki OTA, raportowanie, sygnały Zigbee |
| `main/zigbee_device.h` | Stałe konfiguracyjne: EP, OTA IDs, interwały |
| `main/sensor_task.c` | Jednorazowy task pomiarowy z deep sleep |
| `main/sensor_task.h` | Deklaracja `sensor_task_start()` |
| `main/vl53l0x.c` | Sterownik VL53L0X (level I2C, sekwencja init z ST/Pololu reference) |
| `main/vl53l0x.h` | GPIO i adres I2C czujnika |
| `main/idf_component.yml` | Zależności: esp-zigbee-lib ~1.6.0, esp-zboss-lib ~1.6.0 |
| `z2m/szambo_tof_converter.mjs` | Zewnętrzny konwerter Zigbee2MQTT (ESM format) |
| `ota/index.json` | Indeks OTA dla Z2M (URL, wersja, manufacturer/image type) |
| `tools/make_ota.py` | Skrypt tworzący plik `.zigbee` z binarki firmware |
| `tools/build_ota.sh` | Wrapper shell dla build + make_ota |
| `partitions.csv` | Schemat partycji: dual OTA app + NVS + zb_storage |
| `sdkconfig.defaults` | Konfiguracja ESP-IDF (Zigbee ZED, flash 4MB, deep sleep) |

---

## 3. Endpointy Zigbee i jednostki HA

| EP | Cluster | Kierunek | Jednostka | Opis |
|---|---|---|---|---|
| EP1 | genAnalogInput | read-only | mm | Odległość od czujnika do powierzchni wody |
| EP2 | genAnalogInput | read-only | V | Napięcie baterii |
| EP3 | genAnalogOutput | read/write | min | Interwał pomiarów (1–1440 min) |
| EP4 | genAnalogInput | read-only | % | Poziom napełnienia zbiornika |
| EP5 | genAnalogOutput | read/write | mm | Wysokość zbiornika (dystans sensor→dno przy pełnym) |

EP1 zawiera również klaster **OTA client** (klient aktualizacji firmware).

### Obliczanie fill_level

```c
fill_pct = (tank_height - distance_mm) / tank_height * 100
```

Wartość jest przycinana do `[0, 100]`. Gdy `distance_mm > tank_height` lub
odczyt jest poza zakresem czujnika (VL53L0X zwraca -1), fill_level = 0.

### Encje Home Assistant (po skonfigurowaniu Z2M)

- `sensor.szambo_tof_distance` — odległość w mm
- `sensor.szambo_tof_fill_level` — poziom napełnienia w %
- `sensor.szambo_tof_battery_voltage` — napięcie baterii w V
- `sensor.szambo_tof_battery_level` — poziom baterii w %
- `number.szambo_tof_measurement_interval` — interwał pomiarów (1–1440 min)
- `number.szambo_tof_tank_height` — wysokość zbiornika (100–10000 mm)

---

## 4. Zarządzanie energią — deep sleep

Urządzenie pracuje w trybie **jednego pomiaru na boot + deep sleep**.

```
Boot → Zigbee join/rejoin → pomiar → raport → okno OTA (90s) → deep sleep
```

Typowy cykl przy interwale 60 min:
- **Czas aktywny:** ~95 sekund (join + pomiar + raport + okno komend + OTA check)
- **Czas snu:** ~3505 sekund
- **Szacowany pobór:** ~0.14 mA średni → 550 mAh ≈ **145 dni**

> **WAŻNE:** Light sleep jest wyłączony celowo.
> ESP-Zigbee-SDK v1.4–1.6 ma błąd: włączenie `CONFIG_PM_ENABLE` + `CONFIG_FREERTOS_USE_TICKLESS_IDLE`
> powoduje śmierć radia Zigbee nawet bez jawnego `esp_zb_sleep_enable()`.
> Jedynym bezpiecznym rozwiązaniem jest deep sleep między pomiarami.

### Czas snu (pseudokod)

```c
elapsed = esp_timer_get_time() - t_start;
sleep_us = max(interval_min * 60e6 - elapsed, 55e6);  // min 55 s
esp_deep_sleep_start(sleep_us);
```

---

## 5. OTA — aktualizacja firmware przez sieć

### Jak działa

1. Plik firmware (`.bin`) → skrypt `tools/make_ota.py` → plik `.zigbee`
2. Plik `.zigbee` wgrywany do repozytorium GitHub
3. `ota/index.json` wskazuje URL do pliku i aktualną wersję
4. Zigbee2MQTT pobiera `index.json` przy starcie i zna najnowszą wersję
5. Przy każdym budzeniu urządzenie (EP1 OTA client) wysyła `QueryNextImageRequest`
6. Z2M odpowiada `QueryNextImageResponse` (jeśli jest nowsza wersja)
7. Urządzenie pobiera firmware blokami (64 bajty), zapisuje do partycji OTA
8. Po pobraniu: `esp_ota_end()` → `esp_ota_set_boot_partition()` → `esp_restart()`

### Identyfikatory OTA

| Parametr | Wartość | Hex |
|---|---|---|
| Manufacturer Code | 4097 | `0x1001` |
| Image Type | 4113 | `0x1011` |
| Firmware v1.0.0.9 (bieżąca OTA) | 16777225 | `0x01000009` |

### Tworzenie nowej wersji OTA

```bash
# 1. Zmodyfikuj kod, zmień wersje w plikach:
#    main/zigbee_device.h:  OTA_UPGRADE_FILE_VERSION  (wersja fleszowana via USB)
#    tools/make_ota.py:     FILE_VERSION               (wersja w pliku .zigbee)

# 2. Zbuduj
bash -c 'export IDF_PATH=.../esp-idf-v5.5.2 && \
         eval "$(python3 $IDF_PATH/tools/idf_tools.py export 2>/dev/null)" && \
         idf.py build'

# 3. Utwórz plik OTA
python3 tools/make_ota.py \
    build/szambo_tof_native.bin \
    ota/szambo_tof_native_v1.0.0.X.zigbee

# 4. Zaktualizuj ota/index.json (url, fileVersion)

# 5. Wypchnij na GitHub
git add ota/ && git commit -m "..." && git push

# 6. Zrestartuj Z2M, żeby pobrało nowy index.json
ssh root@172.16.30.23 "docker restart addon_45df7312_zigbee2mqtt"
```

### Wyzwalanie OTA przez MQTT (watcher script)

Urządzenie nie odpytuje serwera OTA samodzielnie w krótkim oknie (timer ZBOSS
domyślnie 24h). Rozwiązanie: Z2M można wyzwolić przez MQTT, co wysyła
`imageNotify`, a urządzenie od razu odpowiada `queryNextImageRequest`.

```sh
# Na Home Assistant: /tmp/ota_watcher.sh
#!/bin/sh
PW="<haslo_z_/config/zigbee2mqtt/configuration.yaml>"
while true; do
    LINE=$(mosquitto_sub -h core-mosquitto -u addons -P "$PW" \
            -t "zigbee2mqtt/bridge/event" -C 1 -W 300 2>&1)
    case "$LINE" in
        *szambo_tof*device_announce*)
            sleep 3
            mosquitto_pub -h core-mosquitto -u addons -P "$PW" \
                -t "zigbee2mqtt/bridge/request/device/ota_update/update" \
                -m '{"id": "szambo_tof"}'
            sleep 120
            ;;
    esac
done

# Uruchom w tle (przeżywa rozłączenie SSH):
setsid /tmp/ota_watcher.sh > /tmp/ota_watcher.log 2>&1 < /dev/null &
```

### Czas transferu OTA

Przy 64 bajtach na blok i ~4 blokach/s:
- Rozmiar firmware: ~559 KB
- Czas transferu: **~46 minut**
- Timeout w firmware: `OTA_TRANSFER_TIMEOUT_US = 5400000000` (90 min)

---

## 6. Budowanie i wgrywanie firmware

### Wymagania

- **ESP-IDF v5.5.2** zainstalowane w `/home/itlk/projekty/archive/esp/esp-idf-v5.5.2`
- Python 3.12

### Budowanie

```bash
cd szambo-tof-native/
bash -c 'export IDF_PATH=/home/itlk/projekty/archive/esp/esp-idf-v5.5.2 && \
         eval "$(python3 $IDF_PATH/tools/idf_tools.py export 2>/dev/null)" && \
         idf.py build'
```

Wynik: `build/szambo_tof_native.bin` (~559 KB, 68% partycji wolne)

### Wgrywanie przez USB na Home Assistant

```bash
# 1. Skopiuj binarki na HA
scp build/szambo_tof_native.bin root@172.16.30.23:/tmp/szambo.bin
scp build/ota_data_initial.bin  root@172.16.30.23:/tmp/ota_data_initial.bin

# 2. Zainstaluj esptool (ginie po każdym restarcie HA OS)
ssh root@172.16.30.23 "pip install esptool --break-system-packages -q"

# 3. Wgraj firmware (TYLKO app + ota_data_initial — NIE erase_flash)
ssh root@172.16.30.23 "esptool --chip esp32c6 --port /dev/ttyACM0 \
    --baud 460800 --before default-reset --after hard-reset \
    write-flash \
    0x10000 /tmp/szambo.bin \
    0x9000  /tmp/ota_data_initial.bin"

# 4. Fizyczny reset (najniezawodniejszy po esptool)
# lub: esptool wykonuje hard-reset automatycznie
```

> **UWAGA:** Nie używaj `erase_flash` — wymaże dane sieci Zigbee (`zb_storage`)
> i ustawienia NVS (interwał, wysokość zbiornika). Urządzenie będzie musiało
> na nowo dołączyć do sieci.

> **UWAGA:** `--before default-reset` wysyła DTR/RTS przez USB CDC do wejścia
> trybu bootloadera. W starszych wersjach esptool to `default_reset` (z podkreśleniem).

### Wgrywanie pierwszego razu (factory flash)

```bash
ssh root@172.16.30.23 "esptool --chip esp32c6 --port /dev/ttyACM0 \
    --baud 460800 --before default-reset --after hard-reset \
    write-flash \
    0x0     build/bootloader/bootloader.bin \
    0x8000  build/partition_table/partition-table.bin \
    0x9000  build/ota_data_initial.bin \
    0x10000 build/szambo_tof_native.bin"
```

---

## 7. Konfiguracja Zigbee2MQTT

### Zewnętrzny konwerter

Skopiuj `z2m/szambo_tof_converter.mjs` do:
```
/config/zigbee2mqtt/external_converters/szambo_tof_converter.mjs
```

> **Wymagania Z2M 2.x:**
> - Format ESM: `export default { ... }` — **NIE** `module.exports = ...`
> - Rozszerzenie `.mjs`
> - Katalog `external_converters/` jest ładowany automatycznie
> - **NIE** dodawaj `external_converters:` do `configuration.yaml` (usunięte w v2.0)

### Indeks OTA

Skopiuj `ota/index.json` do:
```
/config/zigbee2mqtt/ota/index.json
```

Lub dodaj do `configuration.yaml`:
```yaml
ota:
  ikea_ota_use_test_url: false
  # Jeśli Z2M nie pobiera pliku index.json z GitHuba automatycznie:
  # index: /config/zigbee2mqtt/ota/index.json
```

Z2M 2.x automatycznie pobiera index.json z URL podanego w pliku przy starcie.

### Parowanie urządzenia

1. Włącz permit_join w Z2M (max 254 sekund!):
   ```bash
   mosquitto_pub -h core-mosquitto -u addons -P <haslo> \
       -t "zigbee2mqtt/bridge/request/permit_join" \
       -m '{"value": true, "time": 254}'
   ```
2. Włącz urządzenie — przy pierwszym uruchomieniu automatycznie szuka sieci
3. Po sparowaniu Z2M wyśle `device_announce`

### Wymuszenie re-interview (po zmianie endpointów)

```bash
mosquitto_pub -h core-mosquitto -u addons -P <haslo> \
    -t "zigbee2mqtt/bridge/request/device/interview" \
    -m '{"id": "szambo_tof"}'
```

---

## 8. Konfiguracja w Home Assistant

Po skonfigurowaniu Z2M encje pojawiają się automatycznie przez integrację MQTT.

### Ustawienie wysokości zbiornika (jednorazowo)

Przez Z2M MQTT:
```bash
mosquitto_pub -h core-mosquitto -u addons -P <haslo> \
    -t "zigbee2mqtt/szambo_tof/set" \
    -m '{"tank_height": 1740}'
```

Lub przez HA → Urządzenie → `number.szambo_tof_tank_height`.

### Ustawienie interwału pomiarów

Domyślnie: **1 minuta** (tryb testowy).
Dla produkcji ustaw np. **60 minut**:
```bash
mosquitto_pub -h core-mosquitto -u addons -P <haslo> \
    -t "zigbee2mqtt/szambo_tof/set" \
    -m '{"measurement_interval": 60}'
```

Wartość jest zapisywana do NVS i przeżywa reset/OTA.

---

## 9. NVS — trwałe ustawienia

Namespace: `szambo`

| Klucz NVS | Typ | Domyślnie | Zakres | Opis |
|---|---|---|---|---|
| `interval` | uint32 | 1 min | 1–1440 | Interwał pomiarów w minutach |
| `tank_h` | uint32 | 2000 mm | 100–10000 | Wysokość zbiornika w mm |

Dane NVS **przeżywają** reset i OTA (partycja `nvs` jest osobna od `app0`/`app1`).
Dane NVS są kasowane przy `erase_flash` lub ręcznym `nvs_flash_erase()`.

---

## 10. Układ partycji flash

```
Offset      Rozmiar   Nazwa         Typ
0x0000      ~32 KB    bootloader    (poza tabelą partycji)
0x8000      4 KB      part-table    (tabela partycji)
0x9000      8 KB      otadata       data/ota   (aktywna partycja OTA)
0xa000      4 KB      phy_init      data/phy
0x10000     1.75 MB   app0 (ota_0) app/ota_0  (aktywna po factory flash)
0x1c0000    1.75 MB   app1 (ota_1) app/ota_1  (używana przy OTA)
0x370000    436 KB    nvs           data/nvs   (ustawienia NVS)
0x3DC000    16 KB     zb_storage    data/fat   (dane sieci Zigbee)
0x3E0000    1 KB      zb_fct        data/fat   (factory data Zigbee)
```

Firmware (~559 KB) zajmuje ~32% z dostępnych 1.75 MB.

---

## 11. Historia problemów i rozwiązań

### Problem 1: Light sleep niszczy radio Zigbee

**Objawy:** Po kilku minutach urządzenie przestaje odpowiadać na sieć Zigbee.
Nie można go pingować, nie wysyła raportów.

**Przyczyna:** Błąd w ESP-Zigbee-SDK v1.4–1.6: włączenie `CONFIG_PM_ENABLE`
i `CONFIG_FREERTOS_USE_TICKLESS_IDLE` w sdkconfig.defaults (nawet bez jawnego
`esp_zb_sleep_enable(true)`) powoduje nieprawidłowe usypianie radia Zigbee.

**Rozwiązanie:** Całkowite wyłączenie PM light sleep. Zamiast tego używany jest
**deep sleep** między pomiarami (`esp_deep_sleep_start()`). Przy każdym budzeniu
FreeRTOS i cały stos Zigbee startują od nowa.

---

### Problem 2: OTA nie startuje — QueryNextImageRequest nie wysyłany

**Objawy:** Z2M widzi urządzenie (device_announce), ale nie ma żadnej komunikacji
OTA. W logach Z2M brak linii z `queryNextImageRequest`.

**Przyczyna:** Wewnętrzny timer ZBOSS (`timer_counter`) startuje z wartością
**1440** (24 godziny). Wywołanie `esp_zb_ota_upgrade_client_query_interval_set(1)`
zmienia `timer_query` (interwał cykliczny), ale **nie zmienia** aktualnej
wartości `timer_counter`. Timer nigdy nie odpali w 90-sekundowym oknie.

**Rozwiązanie:** Nie polegać na timerze ZBOSS. Zamiast tego Z2M wyzwalamy przez
MQTT wysyłając żądanie OTA update:
```
zigbee2mqtt/bridge/request/device/ota_update/update  {"id": "szambo_tof"}
```
Z2M wysyła wtedy `imageNotify` do urządzenia, które natychmiast odpowiada
`QueryNextImageRequest`. Wyzwalanie odbywa się automatycznie przez skrypt
watcher (`/tmp/ota_watcher.sh` na HA) przy każdym `device_announce`.

---

### Problem 3: OTA kończy się błędem INVALID_IMAGE po pierwszym bloku

**Objawy:** OTA zaczyna się (`estimated 2795s, 11181 chunks`), ale po ułamku
sekundy Z2M loguje `Upgrade end request: INVALID_IMAGE`. Transfer nie kontynuuje.

**Przyczyna (złożona, odkryta empirycznie):**

Format pliku Zigbee OTA (`.zigbee`):
```
[56-bajtowy nagłówek OTA] [sub-element: tag(2B) + length(4B) + dane firmware]
```

ZBOSS w esp-zigbee-lib 1.6.x przekazuje do callbacku `STATUS_RECEIVE` **surowe
dane** licząc od początku sub-elementu, tzn. pierwsze 6 bajtów to:
- bajty 0-1: `0x00 0x00` (tag sub-elementu: `TAG_UPGRADE_IMAGE`)
- bajty 2-5: 4-bajtowa długość firmware (little-endian)

ESP-IDF v5.5.2 `esp_ota_write()` przy pierwszym wywołaniu waliduje **magic byte**
`0xE9` (nagłówek binarki ESP32). Bajt `0x00` ≠ `0xE9` → `ESP_ERR_OTA_VALIDATE_FAILED`
→ callback zwraca `ESP_FAIL` → ZBOSS wysyła status `INVALID_IMAGE` do Z2M.

**Rozwiązanie:** Wykryć i pominąć 6 bajtów nagłówka sub-elementu przy pierwszym
wywołaniu `STATUS_RECEIVE`:

```c
static bool ota_first_block = true;  // resetowane w STATUS_START

// W STATUS_RECEIVE:
if (ota_first_block) {
    ota_first_block = false;
    if (payload_size >= 6 && payload_ptr[0] == 0x00 && payload_ptr[1] == 0x00) {
        payload_ptr += 6;   // pomiń tag(2B) + length(4B)
        payload_size -= 6;
    }
}
esp_ota_write(ota_handle, payload_ptr, payload_size);
```

Warunek `0x00 0x00` jest bezpieczny: jeśli ZBOSS kiedyś zostanie naprawiony
i będzie przekazywał dane już bez nagłówka, pierwsze bajty binarki ESP32 to
`0xE9` (nie `0x00`), więc kod nie pominie nic błędnie.

---

### Problem 4: OTA przerywa się w połowie — restart od 0%

**Objawy:** OTA zaczyna transfer, dochodzi do ~20%, urządzenie idzie spać,
przy kolejnym budzeniu watcher wyzwala OTA ponownie i transfer zaczyna
się od 0%. Nigdy nie dochodzi do 100%.

**Przyczyna:** `OTA_TRANSFER_TIMEOUT_US = 600000000ULL` (10 minut).
Pełny transfer (~559 KB, 4 bloki/s × 64 B = ~46 minut) przekracza timeout.
Po 10 minutach urządzenie wchodzi w deep sleep, zerując stan `esp_ota_begin()`.
Przy następnym budzeniu watcher ponownie wyzwala OTA i cały proces zaczyna od nowa.

**Rozwiązanie:** Zwiększenie timeoutu do 90 minut:
```c
#define OTA_TRANSFER_TIMEOUT_US  5400000000ULL  /* 90 min */
```
Nowa wersja (v1.0.0.8) została wgrana przez USB, a następnie jako OTA image
opublikowana wersja v1.0.0.9. OTA v8→v9 zakończyła się sukcesem (100%,
potwierdzono przez Z2M).

---

### Problem 5: `zb_storage` crash po zmianie liczby endpointów

**Objawy:** Watchdog reset w pętli, urządzenie nie startuje poprawnie.

**Przyczyna:** `zb_storage` (partycja FAT) przechowuje topologię sieci Zigbee
z poprzedniej wersji firmware. Zmiana liczby lub konfiguracji endpointów
powoduje niespójność przy inicjalizacji.

**Rozwiązanie:** Wymagane `erase_flash` przy zmianie endpointów:
```bash
esptool --chip esp32c6 erase_flash
# lub tylko partycja Zigbee:
# esptool write_flash 0x3DC000 <pusty 16KB>
```
Po wymazaniu urządzenie paruje się od nowa.

---

### Problem 6: Z2M nie wczytuje konwertera ESM

**Objawy:** Z2M nie rozpoznaje urządzenia, pojawia się `Not supported (EndDevice)`.

**Przyczyna:** Format CommonJS (`module.exports = ...`) nie jest obsługiwany
w Z2M 2.x. Wymagany jest format ESM.

**Rozwiązanie:**
```javascript
// ZŁE (Z2M <2.0):
module.exports = { zigbeeModel: [...], ... };

// DOBRE (Z2M >=2.0):
export default { zigbeeModel: [...], ... };
```
Plik musi mieć rozszerzenie `.mjs`. Nie dodawać `external_converters:` do
`configuration.yaml` — katalog jest skanowany automatycznie.

---

### Problem 7: ESP32-C6 wchodzi w tryb bootloadera przy otwieraniu portu USB

**Objawy:** Terminal na `/dev/ttyACM0` lub `screen` powoduje reset do ROM
download mode. Urządzenie przestaje działać, miga LEDem.

**Przyczyna:** ESP32-C6 XIAO używa USB-JTAG (CDC-ACM). Otwarcie portu przez
Linux wysyła DTR=1 przez CDC-ACM, co aktywuje układ reset/boot ESP32-C6
i wchodzi w ROM download mode.

**Rozwiązanie:**
- Konsola UART (`sdkconfig`: `CONFIG_ESP_CONSOLE_UART_DEFAULT`) — nie USB
- Podgląd logów przez UART (np. `UART0` na pinach TX/RX XIAO)
- Przy flashowaniu przez USB: użyj `--before default-reset` — celowo wchodzi
  w tryb bootloadera, a po flashowaniu wykonuje hard-reset
- Po flashowaniu: fizyczny reset (wyjęcie/włożenie USB) jest najniezawodniejszy

---

### Problem 8: Skrypt watcher ginie po rozłączeniu SSH

**Objawy:** Uruchomiony `nohup /tmp/ota_watcher.sh &` kończy działanie
po rozłączeniu sesji SSH.

**Przyczyna:** HA OS zabija procesy potomne po zamknięciu sesji SSH.
`nohup` ignoruje `SIGHUP`, ale nie chroni przed `SIGTERM` przy czyszczeniu
grupy procesów.

**Rozwiązanie:** Użyj `setsid` do stworzenia nowej sesji (nowy SID):
```bash
setsid /tmp/ota_watcher.sh > /tmp/ota_watcher.log 2>&1 < /dev/null &
```
Proces ma własny SID, więc nie jest w grupie sesji SSH i przeżywa rozłączenie.

---

### Problem 9: Z2M odrzuca permit_join na czas >254 sekund

**Objawy:** `"time": 300` jest wysyłane, brak potwierdzenia błędu w MQTT,
urządzenie nie może się sparować. Błąd widoczny **tylko** w logach Z2M.

**Przyczyna:** Protokół Zigbee/ZBOSS ogranicza `PermitJoining` do 254 sekund.
Z2M 2.x cicho odrzuca wartości >254 (nie odpowiada błędem przez MQTT).

**Rozwiązanie:** Zawsze używaj `"time": 254` (maksimum):
```bash
mosquitto_pub ... -m '{"value": true, "time": 254}'
```

---

## 12. Schematy pinów

### XIAO ESP32-C6 — użyte GPIO

| GPIO | Funkcja | Uwagi |
|---|---|---|
| 16 | I2C SDA (VL53L0X) | — |
| 17 | I2C SCL (VL53L0X) | — |
| 23 | VL53L0X XSHUT | LOW=on, HIGH=off (power gating) |
| 2 | ADC1_CH2 (battery) | Dzielnik 200k + 100k |
| 12/13 | UART TX/RX (konsola) | Wbudowane w XIAO |

### XIAO ESP32-C6 — **nieużywane / zarezerwowane**

- GPIO0 — boot mode (nie używać jako wyjście)
- USB D+/D- — USB-JTAG CDC (ostrożnie z DTR)

---

## 13. Wersje oprogramowania

| Wersja | OTA file version hex | Co zmieniono |
|---|---|---|
| v1.0.0.1 | `0x01000001` | Pierwsza wersja z OTA |
| v1.0.0.2 | `0x01000002` | — |
| v1.0.0.3 | `0x01000003` | — |
| v1.0.0.4 | `0x01000004` | Test OTA |
| v1.0.0.5 | `0x01000005` | DEFAULT_INTERVAL=1 min (testy) |
| v1.0.0.6 | `0x01000006` | Odczyt interwału przed snem (nie przy budzeniu) |
| v1.0.0.7 | `0x01000007` | timer_query=1min w OTA variable attr |
| v1.0.0.8 | `0x01000008` | **Fix: pominięcie 6-bajtowego nagłówka sub-elementu OTA** |
| v1.0.0.9 | `0x01000009` | **Fix: OTA timeout 10 min → 90 min** (aktualny OTA image) |

### Bieżący stan urządzenia (2026-03-01)

- Firmware wgrany przez USB: **v1.0.0.8** (OTA_UPGRADE_FILE_VERSION = 0x01000008)
- OTA image na GitHub: **v1.0.0.9** (`ota/szambo_tof_native_v1.0.0.9.zigbee`)
- OTA v8→v9 **zakończona sukcesem** o 19:53 CET (potwierdzone przez Z2M)
- Z2M: `installed_version: 16777225` (0x01000009), `state: idle`

### Środowisko budowania

- **ESP-IDF:** v5.5.2 (`/home/itlk/projekty/archive/esp/esp-idf-v5.5.2`)
- **esp-zigbee-lib:** ~1.6.0 (resolves to 1.6.8)
- **esp-zboss-lib:** ~1.6.0 (resolves to 1.6.4)
- **Z2M:** 2.8.0 (coordinator: Sonoff Zigbee 3.0 USB Dongle Plus, zstack)
- **Home Assistant:** SSH `root@172.16.30.23`, Z2M web UI `:8099`
- **Kanał Zigbee:** 11

---

*Dokumentacja wygenerowana 2026-03-01. Repozytorium: https://github.com/kvlh/szambo_zigbee*
