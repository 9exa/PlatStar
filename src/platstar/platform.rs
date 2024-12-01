pub trait Platform {
    type Point;
    
    /// The point on the platform that is closest to the given point
    fn closest_point(&self, p: &Self::Point) -> Self::Point;

    /// The point on the platform that is below the given point, or the closest point if the point is already on the platform
    /// TODO: replace POint with Vector for directional argument
    fn closest_below(&self, down: &Self::Point, p: &Self::Point) -> Self::Point;
}
