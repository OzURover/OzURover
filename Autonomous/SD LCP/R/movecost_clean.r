movecost <- function (dtm, origin, destin, time="s") {

  options(warn = -1)

  altDiff <- function(x){x[2] - x[1]}
  hd <- gdistance::transition(dtm, altDiff, 8, symm=FALSE)

  slope <- gdistance::geoCorrection(hd)

  cost_function <- function(x){ ifelse(x[adj] > 0, 0.01 * exp(-3.5 * abs(x[adj] + 0.05)), 5.3 * exp(-3.5 * abs(x[adj] + 0.05))) }

  adj <- raster::adjacent(dtm, cells=1:ncell(dtm), pairs=TRUE, directions=8)
  speed <- slope
  speed[adj] <- cost_function(slope)
  speed <- speed * 0.278
  Conductance <- gdistance::geoCorrection(speed)
  accum_final <- gdistance::accCost(Conductance, sp::coordinates(origin))

  if (time=="h") {
    accum_final <- accum_final / 3600
  } else if (time=="m") {
    accum_final <- accum_final / 60
  } else {
    accum_final <- accum_final / (3600*24)
  }

  accum_final <- raster::mask(accum_final, dtm)

  sPath <- gdistance::shortestPath(Conductance, sp::coordinates(origin), sp::coordinates(destin), output="SpatialLines")
  sPath$length <- rgeos::gLength(sPath, byid=TRUE)
  destin$cost <- raster::extract(accum_final, destin)
  
  options(warn = 1)

  results <- list("accumulated.cost.raster"=accum_final,
                  "LCPs"=sPath,
                  "dest.loc.w.cost"=destin)
}