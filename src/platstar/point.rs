pub trait Point: PartialEq + Clone {
    fn distance_squared(&self, other: &Self) -> f32;

    fn distance(&self, other: &Self) -> f32 {
        self.distance_squared(other).sqrt()
    }
}

