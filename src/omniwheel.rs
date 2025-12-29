enum OmniWheel {
    Omni_275,
    Omni_325,
    Omni_4,
    Anti_275,
    Anti_325,
    Anti_4,
}

impl OmniWheel {
    fn size(&self) -> f32 {
        match *self {
            OmniWheel::Omni_275 => 2.75,
            OmniWheel::Omni_325 => 3.25,
            OmniWheel::Omni_4 => 4.125,
            OmniWheel::Anti_275 => 2.75,
            OmniWheel::Anti_325 => 3.25,
            OmniWheel::Anti_4 => 4.,
        }
    }
}