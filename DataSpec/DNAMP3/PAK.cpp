#include "PAK.hpp"
#include "DNAMP3.hpp"

namespace DataSpec::DNAMP3 {

const hecl::FourCC CMPD("CMPD");

template <>
void PAK::Enumerate<BigDNA::Read>(athena::io::IStreamReader& reader) {
  m_header.read(reader);
  if (m_header.version != 2)
    Log.report(logvisor::Fatal, fmt("unexpected PAK magic"));

  reader.seek(8, athena::SeekOrigin::Current);
  atUint32 strgSz = reader.readUint32Big();
  reader.seek(4, athena::SeekOrigin::Current);
  atUint32 rshdSz = reader.readUint32Big();
  reader.seek(44, athena::SeekOrigin::Current);
  atUint32 dataOffset = 128 + strgSz + rshdSz;

  atUint64 strgBase = reader.position();
  atUint32 nameCount = reader.readUint32Big();
  m_nameEntries.clear();
  m_nameEntries.reserve(nameCount);
  for (atUint32 n = 0; n < nameCount; ++n) {
    m_nameEntries.emplace_back();
    m_nameEntries.back().read(reader);
  }
  reader.seek(strgBase + strgSz, athena::SeekOrigin::Begin);

  atUint32 count = reader.readUint32Big();
  m_entries.clear();
  m_entries.reserve(count);
  m_firstEntries.clear();
  m_firstEntries.reserve(count);
  std::vector<Entry> entries;
  entries.reserve(count);
  for (atUint32 e = 0; e < count; ++e) {
    entries.emplace_back();
    entries.back().read(reader);
  }
  for (atUint32 e = 0; e < count; ++e) {
    Entry& entry = entries[e];
    entry.offset += dataOffset;

    auto search = m_entries.find(entry.id);
    if (search == m_entries.end()) {
      m_firstEntries.push_back(entry.id);
      m_entries[entry.id] = std::move(entry);
    } else {
      /* Find next MREA to record which area has dupes */
      for (atUint32 e2 = e + 1; e2 < count; ++e2) {
        Entry& entry2 = entries[e2];
        if (entry2.type != FOURCC('MREA'))
          continue;
        m_dupeMREAs.insert(entry2.id);
        break;
      }
    }
  }

  m_nameMap.clear();
  m_nameMap.reserve(nameCount);
  for (NameEntry& entry : m_nameEntries)
    m_nameMap[entry.name] = entry.id;
}

template <>
void PAK::Enumerate<BigDNA::Write>(athena::io::IStreamWriter& writer) {
  m_header.write(writer);

  DNAFourCC("STRG").write(writer);
  atUint32 strgSz = 4;
  for (const NameEntry& entry : m_nameEntries)
    strgSz += (atUint32)entry.name.size() + 13;
  atUint32 strgPad = ((strgSz + 63) & ~63) - strgSz;
  strgSz += strgPad;
  writer.writeUint32Big(strgSz);

  DNAFourCC("RSHD").write(writer);
  atUint32 rshdSz = 4 + 24 * m_entries.size();
  atUint32 rshdPad = ((rshdSz + 63) & ~63) - rshdSz;
  rshdSz += rshdPad;
  writer.writeUint32Big(rshdSz);
  atUint32 dataOffset = 128 + strgSz + rshdSz;

  DNAFourCC("DATA").write(writer);
  atUint32 dataSz = 0;
  for (const auto& entry : m_entries)
    dataSz += (entry.second.size + 63) & ~63;
  atUint32 dataPad = ((dataSz + 63) & ~63) - dataSz;
  dataSz += dataPad;
  writer.writeUint32Big(dataSz);
  writer.seek(36, athena::SeekOrigin::Current);

  writer.writeUint32Big((atUint32)m_nameEntries.size());
  for (const NameEntry& entry : m_nameEntries)
    entry.write(writer);
  writer.seek(strgPad, athena::SeekOrigin::Current);

  writer.writeUint32Big((atUint32)m_entries.size());
  for (const auto& entry : m_entries) {
    Entry copy = entry.second;
    copy.offset -= dataOffset;
    copy.write(writer);
  }
  writer.seek(rshdPad, athena::SeekOrigin::Current);
}

template <>
void PAK::Enumerate<BigDNA::BinarySize>(size_t& __isz) {
  m_header.binarySize(__isz);

  size_t strgSz = 4;
  for (const NameEntry& entry : m_nameEntries)
    strgSz += entry.name.size() + 13;
  size_t strgPad = ((strgSz + 63) & ~63) - strgSz;

  size_t rshdSz = 4 + 24 * m_entries.size();
  size_t rshdPad = ((rshdSz + 63) & ~63) - rshdSz;

  __isz += 60;

  __isz += 4;
  for (const NameEntry& entry : m_nameEntries)
    entry.binarySize(__isz);
  __isz += strgPad;

  __isz += 4;
  for (const auto& entry : m_entries)
    entry.second.binarySize(__isz);
  __isz += rshdPad;
}

std::unique_ptr<atUint8[]> PAK::Entry::getBuffer(const nod::Node& pak, atUint64& szOut) const {
  if (compressed) {
    std::unique_ptr<nod::IPartReadStream> strm = pak.beginReadStream(offset);
    struct {
      hecl::FourCC magic;
      atUint32 blockCount;
    } head;
    strm->read(&head, 8);
    if (head.magic != CMPD) {
      Log.report(logvisor::Error, fmt("invalid CMPD block"));
      return std::unique_ptr<atUint8[]>();
    }
    head.blockCount = hecl::SBig(head.blockCount);

    struct Block {
      atUint32 compSz;
      atUint32 decompSz;
    };
    std::unique_ptr<Block[]> blocks(new Block[head.blockCount]);
    strm->read(blocks.get(), 8 * head.blockCount);

    atUint64 maxBlockSz = 0;
    atUint64 totalDecompSz = 0;
    for (atUint32 b = 0; b < head.blockCount; ++b) {
      Block& block = blocks[b];
      block.compSz = hecl::SBig(block.compSz) & 0xffffff;
      block.decompSz = hecl::SBig(block.decompSz);
      if (block.compSz > maxBlockSz)
        maxBlockSz = block.compSz;
      totalDecompSz += block.decompSz;
    }

    std::unique_ptr<atUint8[]> compBuf(new atUint8[maxBlockSz]);
    atUint8* buf = new atUint8[totalDecompSz];
    atUint8* bufCur = buf;
    for (atUint32 b = 0; b < head.blockCount; ++b) {
      Block& block = blocks[b];
      atUint8* compBufCur = compBuf.get();
      strm->read(compBufCur, block.compSz);
      if (block.compSz == block.decompSz) {
        memcpy(bufCur, compBufCur, block.decompSz);
        bufCur += block.decompSz;
      } else {
        atUint32 rem = block.decompSz;
        while (rem) {
          atUint16 chunkSz = hecl::SBig(*(atUint16*)compBufCur);
          compBufCur += 2;
          size_t dsz;
          lzokay::decompress(compBufCur, chunkSz, bufCur, rem, dsz);
          compBufCur += chunkSz;
          bufCur += dsz;
          rem -= dsz;
        }
      }
    }

    szOut = totalDecompSz;
    return std::unique_ptr<atUint8[]>(buf);
  } else {
    atUint8* buf = new atUint8[size];
    pak.beginReadStream(offset)->read(buf, size);
    szOut = size;
    return std::unique_ptr<atUint8[]>(buf);
  }
}

const PAK::Entry* PAK::lookupEntry(const UniqueID64& id) const {
  auto result = m_entries.find(id);
  if (result != m_entries.end())
    return &result->second;
  return nullptr;
}

const PAK::Entry* PAK::lookupEntry(std::string_view name) const {
  auto result = m_nameMap.find(name.data());
  if (result != m_nameMap.end()) {
    auto result1 = m_entries.find(result->second);
    if (result1 != m_entries.end())
      return &result1->second;
  }
  return nullptr;
}

std::string PAK::bestEntryName(const nod::Node& pakNode, const Entry& entry, std::string& catalogueName) const {
  /* Prefer named entries first */
  for (const NameEntry& nentry : m_nameEntries)
    if (nentry.id == entry.id) {
      catalogueName = nentry.name;
      return fmt::format(fmt("{}_{}"), nentry.name, entry.id);
    }

  /* Otherwise return ID format string */
  return fmt::format(fmt("{}_{}"), entry.type, entry.id);
}

} // namespace DataSpec::DNAMP3
