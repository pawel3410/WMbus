/*
 Copyright (C) 2019 Jacek Tomasiak (gpl-3.0-or-later)
 Copyright (C) 2020-2023 Fredrik Öhrström (gpl-3.0-or-later)
 Copyright (C) 2021 Vincent Privat (gpl-3.0-or-later)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "meters_common_implementation.h"
#include "manufacturer_specificities.h"

namespace
{
    /** Contains all the booleans required to store the alarms of a PRIOS device. */
    struct IzarAlarms
    {
        bool general_alarm;
        bool leakage_currently;
        bool leakage_previously;
        bool meter_blocked;
        bool back_flow;
        bool underflow;
        bool overflow;
        bool submarine;
        bool sensor_fraud_currently;
        bool sensor_fraud_previously;
        bool mechanical_fraud_currently;
        bool mechanical_fraud_previously;
    };

    struct Driver : public virtual MeterCommonImplementation
    {
        Driver(MeterInfo &mi, DriverInfo &di);

        void processContent(Telegram *t);

    private:

        std::string currentAlarmsText(IzarAlarms &alarms);
        std::string previousAlarmsText(IzarAlarms &alarms);

        std::vector<uchar> decodePrios(const std::vector<uchar> &origin, const std::vector<uchar> &payload, uint32_t key);

        std::vector<uint32_t> keys;
    };

    static bool ok = registerDriver([](DriverInfo& di)
    {
        di.setName("izar2");
        di.setDefaultFields("name,id,prefix,serial_number,total_m3,last_month_total_m3,"
                            "last_month_measure_date,"
                            "remaining_battery_life_y,"
                            "current_alarms,"
                            "previous_alarms,"
                            "transmit_period_s,"
                            "manufacture_year,timestamp");
        di.setMeterType(MeterType::WaterMeter);
        di.addLinkMode(LinkMode::T1);
        // Obsługa nowych modułów Diehl/IZAR raportujących jako MAD (0x3424)
        di.addDetection(MANUFACTURER_MAD, 0x20, -1);
        di.addDetection(MANUFACTURER_MAD, 0xA2, -1);
        di.addDetection(MANUFACTURER_MAD, 0xA1, -1);
        di.usesProcessContent();

        di.setConstructor([](MeterInfo& mi, DriverInfo& di){ return std::shared_ptr<Meter>(new Driver(mi, di)); });
    });

    Driver::Driver(MeterInfo &mi, DriverInfo &di) : MeterCommonImplementation(mi, di)
    {
        initializeDiehlDefaultKeySupport(meterKeys()->confidentiality_key, keys);

        addStringField("prefix",
                       "The alphanumeric prefix printed before serial number on device.",
                       DEFAULT_PRINT_PROPERTIES);

        addStringField("serial_number",
                       "The meter serial number.",
                       DEFAULT_PRINT_PROPERTIES);

        addNumericField("total",
                        Quantity::Volume,
                        DEFAULT_PRINT_PROPERTIES,
                        "The total water consumption recorded by this meter.");

        addNumericField("last_month_total",
                        Quantity::Volume,
                        DEFAULT_PRINT_PROPERTIES,
                        "The total water consumption recorded by this meter around end of last month.");

        addStringField("last_month_measure_date",
                       "The date when the meter recorded the most recent billing value.",
                       DEFAULT_PRINT_PROPERTIES);

        addNumericField("remaining_battery_life",
                        Quantity::Time,
                        DEFAULT_PRINT_PROPERTIES,
                        "How many more years the battery is expected to last",
                        Unit::Year);

        addStringField("current_alarms",
                       "Alarms currently reported by the meter.",
                       DEFAULT_PRINT_PROPERTIES);

        addStringField("previous_alarms",
                       "Alarms previously reported by the meter.",
                       DEFAULT_PRINT_PROPERTIES);

        addNumericField("transmit_period", Quantity::Time,
                        DEFAULT_PRINT_PROPERTIES,
                        "The period at which the meter transmits its data.",
                        Unit::Second);

        addStringField("manufacture_year",
                       "The year during which the meter was manufactured.",
                       DEFAULT_PRINT_PROPERTIES);
    }

    std::string Driver::currentAlarmsText(IzarAlarms &alarms)
    {
        std::string s;
        if (alarms.leakage_currently) {
            s.append("leakage,");
        }
        if (alarms.meter_blocked) {
            s.append("meter_blocked,");
        }
        if (alarms.back_flow) {
            s.append("back_flow,");
        }
        if (alarms.underflow) {
            s.append("underflow,");
        }
        if (alarms.overflow) {
            s.append("overflow,");
        }
        if (alarms.submarine) {
            s.append("submarine,");
        }
        if (alarms.sensor_fraud_currently) {
            s.append("sensor_fraud,");
        }
        if (alarms.mechanical_fraud_currently) {
            s.append("mechanical_fraud,");
        }
        if (s.length() > 0) {
            if (alarms.general_alarm) {
                return "general_alarm";
            }
            s.pop_back();
            return s;
        }
        return "no_alarm";
    }

    std::string Driver::previousAlarmsText(IzarAlarms &alarms)
    {
        std::string s;
        if (alarms.leakage_previously) {
            s.append("leakage,");
        }
        if (alarms.sensor_fraud_previously) {
            s.append("sensor_fraud,");
        }
        if (alarms.mechanical_fraud_previously) {
            s.append("mechanical_fraud,");
        }
        if (s.length() > 0) {
            s.pop_back();
            return s;
        }
        return "no_alarm";
    }

    void Driver::processContent(Telegram *t)
    {
        std::vector<uchar> frame;
        t->extractFrame(&frame);
        std::vector<uchar> origin = t->original.empty() ? frame : t->original;

        // Pre-processing / rotacja pola adresowego dla nowych ramek MAD
        if (origin.size() >= 10) {
            uchar temp0 = origin[4];
            uchar temp1 = origin[5];
            origin[4] = origin[6];
            origin[5] = origin[7];
            origin[6] = origin[8];
            origin[7] = origin[9];
            origin[8] = temp0;
            origin[9] = temp1;
        }
        if (frame.size() >= 10) {
            uchar temp0 = frame[4];
            uchar temp1 = frame[5];
            frame[4] = frame[6];
            frame[5] = frame[7];
            frame[6] = frame[8];
            frame[7] = frame[9];
            frame[8] = temp0;
            frame[9] = temp1;
        }

        std::vector<uchar> decoded_content;
        for (auto& key : keys) {
            decoded_content = decodePrios(origin, frame, key);
            if (!decoded_content.empty())
                break;
        }

        debug("(izar2) Decoded PRIOS data: %s\n", bin2hex(decoded_content).c_str());

        if (decoded_content.empty())
        {
            if (!t->beingAnalyzed())
            {
                warning("(izar2) Decoding PRIOS data failed. Ignoring telegram.\n");
            }
            return;
        }

        // Odczyt metadanych z wyrównanego pola adresowego
        {
            std::string digits = std::to_string((origin[7] & 0x03) << 24 | origin[6] << 16 | origin[5] << 8 | origin[4]);
            digits = tostrprintf("%08d", atoi(digits.c_str())); 
            uint8_t yy = atoi(digits.substr(0, 2).c_str());
            int manufacture_year = yy > 70 ? (1900 + yy) : (2000 + yy);
            setStringValue("manufacture_year", tostrprintf("%d", manufacture_year), NULL);

            uint32_t serial_number = atoi(digits.substr(2, digits.size()).c_str());
            setStringValue("serial_number", tostrprintf("%06d", serial_number), NULL);

            uchar supplier_code = '@' + (((origin[9] & 0x0F) << 1) | (origin[8] >> 7));
            uchar meter_type = '@' + ((origin[8] & 0x7C) >> 2);
            uchar diameter = '@' + (((origin[8] & 0x03) << 3) | (origin[7] >> 5));
            std::string prefix = tostrprintf("%c%02d%c%c", supplier_code, yy, meter_type, diameter);
            setStringValue("prefix", prefix, NULL);
        }

        // get the remaining battery life (in year) and transmission period (in seconds)
        double remaining_battery_life = (frame[12] & 0x1F) / 2.0;
        setNumericValue("remaining_battery_life", Unit::Year, remaining_battery_life);

        int transmit_period_s = 1 << ((frame[11] & 0x0F) + 2);
        setNumericValue("transmit_period", Unit::Second, transmit_period_s);

        double total_water_consumption_l_ = uint32FromBytes(decoded_content, 1, true);
        setNumericValue("total", Unit::L, total_water_consumption_l_);

        if (decoded_content.size() > 8) {
            double last_month_total_water_consumption_l_ = uint32FromBytes(decoded_content, 5, true);
            setNumericValue("last_month_total", Unit::L, last_month_total_water_consumption_l_);
        }
        
        // get the date when the second measurement was taken
        if (decoded_content.size() > 10) {
            uint16_t h0_year = ((decoded_content[10] & 0xF0) >> 1) + ((decoded_content[9] & 0xE0) >> 5);
            if (h0_year > 80) {
                h0_year += 1900;
            } else {
                h0_year += 2000;
            }
            uint8_t h0_month = decoded_content[10] & 0xF;
            uint8_t h0_day = decoded_content[9] & 0x1F;

            setStringValue("last_month_measure_date", tostrprintf("%d-%02d-%02d", h0_year, h0_month%99, h0_day%99), NULL);
        }

        // read the alarms:
        IzarAlarms alarms {};

        alarms.general_alarm = frame[11] >> 7;
        alarms.leakage_currently = frame[12] >> 7;
        alarms.leakage_previously = frame[12] >> 6 & 0x1;
        alarms.meter_blocked = frame[12] >> 5 & 0x1;
        alarms.back_flow = frame[13] >> 7;
        alarms.underflow = frame[13] >> 6 & 0x1;
        alarms.overflow = frame[13] >> 5 & 0x1;
        alarms.submarine = frame[13] >> 4 & 0x1;
        alarms.sensor_fraud_currently = frame[13] >> 3 & 0x1;
        alarms.sensor_fraud_previously = frame[13] >> 2 & 0x1;
        alarms.mechanical_fraud_currently = frame[13] >> 1 & 0x1;
        alarms.mechanical_fraud_previously = frame[13] & 0x1;

        setStringValue("current_alarms", currentAlarmsText(alarms));
        setStringValue("previous_alarms", previousAlarmsText(alarms));
    }

    std::vector<uchar> Driver::decodePrios(const std::vector<uchar> &origin, const std::vector<uchar> &frame, uint32_t key)
    {
        return decodeDiehlLfsr(origin, frame, key, DiehlLfsrCheckMethod::HEADER_1_BYTE, 0x4B);
    }
}

KEEP_DRIVER(izar2);
