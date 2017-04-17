#include <utility>

// Pair hash lets us do hash-based maps and sets with std::pair
// e.g. std::unordered_set<std::pair<unsigned int, unsigned int>, pairhash>
// partially borrowed from: http://stackoverflow.com/a/20602159/6153561
// modified to improve hashing collision avoidance
// magic number stolen from boost hash_combine
struct pairhash {
public:
	template <typename T, typename U>
	std::size_t operator()(const std::pair<T, U> &x) const

	{
		return std::hash<T>()(x.first) ^ (std::hash<U>()(x.second) + 0x9e3779b9);
	}
};
