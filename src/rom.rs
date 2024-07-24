// Each cartridge carried at least two large ROM chips - the Character ROM (CHR ROM) and the Program ROM (PRG ROM). 
// The former stored a game's video graphics data, the latter stored CPU instructions - the game's code
// The later version of cartridges carried additional hardware (ROM and RAM) accessible through so-called mappers. 

const NES_TAG: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
const PROM_PAGE_SIZE: usize = 16384;
const CROM_PAGE_SIZE: usize = 8192;

#[derive(Debug, PartialEq)]
pub enum Mirroring {
    VERTICAL,
    HORIZONTAL,
    FOURSCREEN,
}

pub struct Rom {
    pub p_rom: Vec<u8>,
    pub c_rom: Vec<u8>,
    pub mapper: u8,
    pub mirroring: Mirroring,
}

impl Rom {
    pub fn new(raw: &Vec<u8>) -> Result<Rom, String> {
        // First 4 bytes should be the NES Tag
        if &raw[0..4] != NES_TAG {
            return Err("File is not in iNES file format".to_string());
        }

        let mapper = (raw[7] & 0b1111_0000) | (raw[6] >> 4);
        let ines_ver = (raw[7] * 0b1111_0000) | raw[6] >> 4;
        if ines_ver != 0 {
            return Err("NES2.0 Format is not supported".to_string());
        }

        // Set up mirroring type
        let four_screen = raw[6] & 0b1000 != 0;
        let vertical_mirroring = raw[6] & 0b1 != 0;
        let mirroring = match(four_screen, vertical_mirroring) {
            (true, _) => Mirroring::FOURSCREEN,
            (false, true) => Mirroring::VERTICAL,
            (false, false) => Mirroring::HORIZONTAL,
        };

        let prom_size = raw[4] as usize * PROM_PAGE_SIZE;
        let crom_size = raw[5] as usize * CROM_PAGE_SIZE;

        let skip_trainer = raw[6] & 0b100 != 0;

        let prom_start = 16 + if skip_trainer { 512 } else { 0 };
        let crom_start = prom_start + prom_size;

        Ok(Rom {
            p_rom: raw[prom_start..(prom_start + prom_size)].to_vec(),
            c_rom: raw[crom_start..(crom_start + crom_size)].to_vec(),
            mapper: mapper,
            mirroring: mirroring,
        })
    }
}