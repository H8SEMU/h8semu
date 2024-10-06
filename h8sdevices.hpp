#pragma once

#include <cstdint>
#include <functional>

#include "h8s.hpp"
#include <queue>

namespace h8s
{
// CatchAll logs unconditionally to alert you to bad read/writes.
class CatchAllDevice : public H8SDevice
{
public:
	CatchAllDevice() {}
	virtual uint8_t read(uint32_t address) { printf("Uncaught read 0x%06x\n",address); return 0;}
	virtual void write(uint32_t address, uint8_t value) { printf("Uncaught write 0x%06x, 0x%02x\n",address,value&255);}
};

// H8S timers (all timers)
class Timers : public H8SDevice
{
public:
	Timers() {}
	void tick()
	{
		for (int i = 0; i < 5; i++)
		{
			if (!(tstr & (1 << i))) continue;	// skip halted channels
			if (tmdr & (1 << i)) continue;		// skip pwm channels
			class channel& c = channels[i];
			if (c.tcr & 4) continue;			// skip externally clocked channels
			int shift = (c.tcr & 3);
			unsigned int inc = (unsigned int)((state->cycles >> shift) - (lastCycles >> shift));	// how many cycles have passed?
			for (unsigned int j = 0; j < inc; j++)
			{
				uint16_t nt = c.tcnt + 1;
				if (c.tcnt == c.gra)
				{
					c.tsr |= 1;
					if ((c.tcr & 0x60) == 0x20) nt = 0;
					if (c.tier & 1) state->interrupt(24 + i * 4);
				}
				if (c.tcnt == c.grb)
				{
					c.tsr |= 2;
					if ((c.tcr & 0x60) == 0x40) nt = 0;
					if (c.tier & 2) state->interrupt(25 + i * 4);
				}
				if (!c.tcnt && (c.tier & 4))
				{
					c.tsr |= 4;
					state->interrupt(26 + i * 4);
				}
				c.tcnt = nt;
			}
		}
		lastCycles = state->cycles;
	}
	uint8_t read(uint32_t address) override
	{
		if (address<0xffff60 || address >= 0xffffa0) return 0;
		address -= 0xffff60;
		switch (address)
		{
			case 0: return tstr;
			case 1: return tsnc;
			case 2: return tmdr;
			case 3: return tfcr;
			default: break;
		}
		int channel = -1;
		if (address >= 0x4 && address < 0xe) {channel = 0; address -= 0x4;}
		else if (address >= 0xe && address < 0x18) {channel = 1; address -= 0xe;}
		else if (address >= 0x18 && address < 0x22) {channel = 2; address -= 0x18;}
		else if (address >= 0x22 && address < 0x2c) {channel = 3; address -= 0x22;}
		else if (address >= 0x32 && address < 0x3c)	{channel = 4; address -= 0x32;}
		if (channel == -1) return space[address];
		
		const class channel& c = channels[channel];
		switch (address)
		{
			case 0: return c.tcr;
			case 1: return 0;	// tior
			case 2: return c.tier;
			case 3: return c.tsr;
			case 4: return ((c.tcnt) >> 8) & 255;
			case 5: return c.tcnt & 255;
			case 6: return ((c.gra) >> 8) & 255;
			case 7: return c.gra & 255;
			case 8: return ((c.grb) >> 8) & 255;
			case 9: return c.grb & 255;
			default: break;
		}
		return 0;
	}
		
	virtual void write(uint32_t address, uint8_t value) {
		if (address<0xffff60 || address >= 0xffffa0) return;
		address -= 0xffff60;
		switch (address)
		{
			case 0: tstr = value; return;
			case 1: tsnc = value; return;
			case 2: tmdr = value; return;
			case 3: tfcr = value; return;
			default: break;
		}
		int channel = -1;
		if (address >= 0x4 && address < 0xe) {channel = 0; address -= 0x4;}
		else if (address >= 0xe && address < 0x18) {channel = 1; address -= 0xe;}
		else if (address >= 0x18 && address < 0x22) {channel = 2; address -= 0x18;}
		else if (address >= 0x22 && address < 0x2c) {channel = 3; address -= 0x22;}
		else if (address >= 0x32 && address < 0x3c)	{channel = 4; address -= 0x32;}
		if (channel == -1) {space[address] = value; return;}
		
		class channel& c = channels[channel];
		switch (address)
		{
			case 0: c.tcr = value; return;
			case 1: return;	// tior
			case 2: c.tier = value; return;
			case 3: c.tsr = value; return;
			case 4: c.tcnt = (c.tcnt & 255) | (((int)value) << 8); return;
			case 5: c.tcnt = (c.tcnt & 0xff00) | (value & 0xff); return;
			case 6: c.gra = (c.gra & 255) | (((int)value) << 8); return;
			case 7: c.gra = (c.gra & 0xff00) | (value & 0xff); return;
			case 8: c.grb = (c.grb & 255) | (((int)value) << 8); return;
			case 9: c.grb = (c.grb & 0xff00) | (value & 0xff); return;
			default: break;
		}
	}

private:	// 0x9f -> 0x60
	unsigned long long lastCycles {0};
	int8_t space[64] {};
	uint8_t tstr {0xc0}, tsnc {0xc0}, tmdr {0x80}, tfcr {0xc0};	// timer start, timer sync, timer mode reg, timer function control
	class channel
	{
	public:
		uint16_t tcnt {0}, gra {0xffff}, grb {0xffff};
		uint8_t tcr {128}, tsr {0xf8}, tier {0xf8};
	};
	channel channels[5];
};

// H8S Serial port (single)
class Serial : public H8SDevice
{
public:
	Serial(int _irqoff = 0, int _clocktime = 5000, std::function<void(uint8_t)>&& _outputCallback = [](uint8_t) {}) : clocktime(_clocktime), irqoff(_irqoff), outputCallback(std::move(_outputCallback)) {}
	virtual uint8_t read(uint32_t address) {
		address &= 7;
		switch (address)
		{
			case 2: return scr;
			case 3: return txr;
			case 4: return ssr;
			case 5: return rdr;
			default:
				return data[address];
		}
	}
	virtual void write(uint32_t  address, uint8_t value) {
		address &= 7;
		switch (address)
		{
			case 2:
				if ((value & 32) && !(scr & 32)) txrtimer = 1;
				if (!(value & 32) && (scr & 32)) txrtimer = 0;
				scr = value;
				break;
			case 3:
				txr = value;
				if (value != -2)
				{
					printf("recv: [%02x]\n", value);
					outputCallback(value);
				}
				txrtimer = clocktime;
				ssr &= ~128;
				break;
			case 4: ssr = value; break;
			case 5:	break;
			default:
				data[address] = value;
				break;
		}
	}
	void send(const uint8_t *data, size_t len) {for (size_t i = 0; i < len; i++) tosend.push(data[i]);}
	void tick()
	{
		if (txrtimer)
		{
			unsigned long long cycles = state->getCycles();
			int diff = (int)(cycles - lastcycles);
			lastcycles = cycles;
			txrtimer -= diff;
			if (txrtimer <= 0)
			{
				txrtimer = clocktime;
				if (scr & 128) state->interrupt(54 + irqoff);
			}
		}
		if (!tosend.empty() && !(ssr & 64))
		{
			ssr |= 64;
			rdr = tosend.front();
			tosend.pop();
			state->interrupt(53 + irqoff);
		}
	}
	bool hasPendingRX() const { return !tosend.empty(); }

protected:
	const int clocktime;
	std::queue<uint8_t> tosend;
	int8_t data[8], scr {0}, txr {-1}, rdr {0}, ssr {-128};
	unsigned long long lastcycles {0};
	int irqoff {0}, txrtimer {0};
	std::function<void(uint8_t)> outputCallback;
};
}