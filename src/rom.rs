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

pub mod test {

    use super::*;

    struct TestRom {
        header: Vec<u8>,
        trainer: Option<Vec<u8>>,
        pgp_rom: Vec<u8>,
        chr_rom: Vec<u8>,
    }

    fn create_rom(rom: TestRom) -> Vec<u8> {
        let mut result = Vec::with_capacity(
            rom.header.len()
                + rom.trainer.as_ref().map_or(0, |t| t.len())
                + rom.pgp_rom.len()
                + rom.chr_rom.len(),
        );

        result.extend(&rom.header);
        if let Some(t) = rom.trainer {
            result.extend(t);
        }
        result.extend(&rom.pgp_rom);
        result.extend(&rom.chr_rom);

        result
    }

    pub fn test_rom() -> Rom {
        let test_rom = create_rom(TestRom {
            header: vec![
                0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x31, 00, 00, 00, 00, 00, 00, 00, 00, 00,
            ],
            trainer: None,
            pgp_rom: vec![1; 2 * PROM_PAGE_SIZE],
            chr_rom: vec![2; 1 * CROM_PAGE_SIZE],
        });

        Rom::new(&test_rom).unwrap()
    }

    #[test]
    fn test() {
        let test_rom = create_rom(TestRom {
            header: vec![
                0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x31, 00, 00, 00, 00, 00, 00, 00, 00, 00,
            ],
            trainer: None,
            pgp_rom: vec![1; 2 * PROM_PAGE_SIZE],
            chr_rom: vec![2; 1 * CROM_PAGE_SIZE],
        });

        let rom: Rom = Rom::new(&test_rom).unwrap();

        assert_eq!(rom.c_rom, vec!(2; 1 * CROM_PAGE_SIZE));
        assert_eq!(rom.p_rom, vec!(1; 2 * PROM_PAGE_SIZE));
        assert_eq!(rom.mapper, 3);
        assert_eq!(rom.mirroring, Mirroring::VERTICAL);
    }

    #[test]
    fn test_with_trainer() {
        let test_rom = create_rom(TestRom {
            header: vec![
                0x4E,
                0x45,
                0x53,
                0x1A,
                0x02,
                0x01,
                0x31 | 0b100,
                00,
                00,
                00,
                00,
                00,
                00,
                00,
                00,
                00,
            ],
            trainer: Some(vec![0; 512]),
            pgp_rom: vec![1; 2 * PROM_PAGE_SIZE],
            chr_rom: vec![2; 1 * CROM_PAGE_SIZE],
        });

        let rom: Rom = Rom::new(&test_rom).unwrap();

        assert_eq!(rom.c_rom, vec!(2; 1 * CROM_PAGE_SIZE));
        assert_eq!(rom.p_rom, vec!(1; 2 * PROM_PAGE_SIZE));
        assert_eq!(rom.mapper, 3);
        assert_eq!(rom.mirroring, Mirroring::VERTICAL);
    }

    #[test]
    fn test_nes2_is_not_supported() {
        let test_rom = create_rom(TestRom {
            header: vec![
                0x4E, 0x45, 0x53, 0x1A, 0x01, 0x01, 0x31, 0x8, 00, 00, 00, 00, 00, 00, 00, 00,
            ],
            trainer: None,
            pgp_rom: vec![1; 1 * PROM_PAGE_SIZE],
            chr_rom: vec![2; 1 * CROM_PAGE_SIZE],
        });
        let rom = Rom::new(&test_rom);
        match rom {
            Result::Ok(_) => assert!(false, "should not load rom"),
            Result::Err(str) => assert_eq!(str, "NES2.0 format is not supported"),
        }
    }
}