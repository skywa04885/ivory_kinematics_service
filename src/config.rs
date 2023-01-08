#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    TomlDe(toml::de::Error),
}

impl From<std::io::Error> for Error {
    fn from(value: std::io::Error) -> Self {
        Error::Io(value)
    }
}

impl From<toml::de::Error> for Error {
    fn from(value: toml::de::Error) -> Self {
        Error::TomlDe(value)
    }
}

#[derive(serde_derive::Deserialize)]
pub struct Config {
    pub rabbitmq: RabbitMQ,
}

#[derive(serde_derive::Deserialize)]
pub struct RabbitMQ {
    pub uri: String,
    pub exchange: String,
}

impl Config {
    pub fn from_file(file: std::borrow::Cow<'static, str>) -> Result<Config, Error> {
        let file_buffer: Vec<u8> = std::fs::read(file.as_ref())?;
        let config: Config = toml::from_slice::<Config>(&file_buffer)?;

        Ok(config)
    }
}
