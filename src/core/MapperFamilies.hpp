#pragma once
#include <memory>
#include "Mapper.hpp"

std::unique_ptr<Mapper> createNintendoDiscreteMapper(const MapperConfig& config);

std::unique_ptr<Mapper> createMmc1Mapper(const MapperConfig& config);
std::unique_ptr<Mapper> createMmc3Mapper(const MapperConfig& config);

std::unique_ptr<Mapper> createMmc5Mapper(const MapperConfig& config);
std::unique_ptr<Mapper> createNamcoMapper(const MapperConfig& config);

std::unique_ptr<Mapper> createVrcMapper(const MapperConfig& config);
std::unique_ptr<Mapper> createSunsoftMapper(const MapperConfig& config);

std::unique_ptr<Mapper> createUnlicensedMapper(const MapperConfig& config);
