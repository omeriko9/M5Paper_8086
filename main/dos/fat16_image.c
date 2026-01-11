/**
 * @file fat16_image.c
 * @brief Minimal FAT16 image read/write (8.3 only) for WebUI file manager.
 */

#include "fat16_image.h"

#include <ctype.h>
#include <string.h>
#include <time.h>

typedef struct fat16_image
{
  FILE *f;
  bool writeable;
  char path[96];

  uint32_t part_lba;
  uint32_t total_sectors;
  uint16_t bytes_per_sector;
  uint8_t sectors_per_cluster;
  uint16_t reserved_sectors;
  uint8_t fats;
  uint16_t root_entries;
  uint16_t sectors_per_fat;

  uint32_t fat_lba;
  uint32_t root_lba;
  uint32_t root_sectors;
  uint32_t data_lba;

  uint16_t alloc_hint;
} fat16_image_t;

static uint16_t read_le16(const uint8_t *buf)
{
  return (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
}

static uint32_t read_le32(const uint8_t *buf)
{
  return (uint32_t)(buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
}

static void write_le16(uint8_t *buf, uint16_t v)
{
  buf[0] = (uint8_t)(v & 0xFF);
  buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void write_le32(uint8_t *buf, uint32_t v)
{
  buf[0] = (uint8_t)(v & 0xFF);
  buf[1] = (uint8_t)((v >> 8) & 0xFF);
  buf[2] = (uint8_t)((v >> 16) & 0xFF);
  buf[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void set_err(char *err, size_t err_len, const char *msg)
{
  if (!err || err_len == 0)
    return;
  snprintf(err, err_len, "%s", msg ? msg : "error");
}

static bool img_read_sector(fat16_image_t *img, uint32_t lba, uint8_t *out)
{
  if (!img || !img->f || !out)
    return false;
  if (fseek(img->f, (long)(lba * 512u), SEEK_SET) != 0)
    return false;
  return fread(out, 1, 512, img->f) == 512;
}

static bool img_write_sector(fat16_image_t *img, uint32_t lba, const uint8_t *in)
{
  if (!img || !img->f || !in || !img->writeable)
    return false;
  if (fseek(img->f, (long)(lba * 512u), SEEK_SET) != 0)
    return false;
  return fwrite(in, 1, 512, img->f) == 512;
}

static uint32_t cluster_to_lba(const fat16_image_t *img, uint16_t cluster)
{
  if (!img || cluster < 2)
    return 0;
  return img->data_lba + (uint32_t)(cluster - 2u) * (uint32_t)img->sectors_per_cluster;
}

static bool fat_get(fat16_image_t *img, uint16_t cluster, uint16_t *out)
{
  if (!img || !out)
    return false;
  uint32_t off = (uint32_t)cluster * 2u;
  uint32_t lba = img->fat_lba + (off / 512u);
  uint16_t boff = (uint16_t)(off % 512u);
  uint8_t sec[512];
  if (!img_read_sector(img, lba, sec))
    return false;
  *out = read_le16(sec + boff);
  return true;
}

static bool fat_set_one(fat16_image_t *img, uint32_t fat_lba, uint16_t cluster, uint16_t val)
{
  uint32_t off = (uint32_t)cluster * 2u;
  uint32_t lba = fat_lba + (off / 512u);
  uint16_t boff = (uint16_t)(off % 512u);
  uint8_t sec[512];
  if (!img_read_sector(img, lba, sec))
    return false;
  write_le16(sec + boff, val);
  return img_write_sector(img, lba, sec);
}

static bool fat_set(fat16_image_t *img, uint16_t cluster, uint16_t val)
{
  if (!img || !img->writeable)
    return false;
  for (uint8_t i = 0; i < img->fats; i++)
  {
    uint32_t fat_lba = img->fat_lba + (uint32_t)i * (uint32_t)img->sectors_per_fat;
    if (!fat_set_one(img, fat_lba, cluster, val))
      return false;
  }
  return true;
}

static bool is_fat_eoc(uint16_t v)
{
  return v >= 0xFFF8u;
}

static bool name83_from_component(const char *in, char out11[11], char *err, size_t err_len)
{
  if (!in || !out11)
  {
    set_err(err, err_len, "invalid name");
    return false;
  }

  const char *dot = strchr(in, '.');
  size_t base_len = dot ? (size_t)(dot - in) : strlen(in);
  size_t ext_len = dot ? strlen(dot + 1) : 0;
  if (base_len == 0 || base_len > 8 || ext_len > 3)
  {
    set_err(err, err_len, "name must be 1-8 chars, ext 0-3");
    return false;
  }

  memset(out11, ' ', 11);
  for (size_t i = 0; i < base_len; i++)
  {
    unsigned char ch = (unsigned char)in[i];
    if (ch <= 0x20 || ch >= 0x7F || ch == '.' || ch == '/' || ch == '\\')
    {
      set_err(err, err_len, "invalid character");
      return false;
    }
    out11[i] = (char)toupper(ch);
  }
  for (size_t i = 0; i < ext_len; i++)
  {
    unsigned char ch = (unsigned char)dot[1 + i];
    if (ch <= 0x20 || ch >= 0x7F || ch == '.' || ch == '/' || ch == '\\')
    {
      set_err(err, err_len, "invalid character");
      return false;
    }
    out11[8 + i] = (char)toupper(ch);
  }

  if ((unsigned char)out11[0] == 0xE5)
  {
    out11[0] = '_';
  }

  return true;
}

static void name83_to_string(const uint8_t name11[11], char out[13])
{
  char base[9];
  char ext[4];
  memcpy(base, name11, 8);
  base[8] = '\0';
  memcpy(ext, name11 + 8, 3);
  ext[3] = '\0';

  for (int i = 7; i >= 0 && base[i] == ' '; i--)
    base[i] = '\0';
  for (int i = 2; i >= 0 && ext[i] == ' '; i--)
    ext[i] = '\0';

  if (ext[0])
  {
    snprintf(out, 13, "%s.%s", base, ext);
  }
  else
  {
    snprintf(out, 13, "%s", base);
  }
}

typedef struct
{
  bool is_root;
  uint16_t first_cluster;
} dir_loc_t;

typedef struct
{
  uint32_t lba;
  uint16_t off;
} dirent_ref_t;

static bool read_dirent_at(fat16_image_t *img, uint32_t lba, uint16_t off, uint8_t out_ent[32])
{
  uint8_t sec[512];
  if (!img_read_sector(img, lba, sec))
    return false;
  if (off > (512 - 32))
    return false;
  memcpy(out_ent, sec + off, 32);
  return true;
}

static bool write_dirent_at(fat16_image_t *img, uint32_t lba, uint16_t off, const uint8_t ent[32])
{
  uint8_t sec[512];
  if (!img_read_sector(img, lba, sec))
    return false;
  if (off > (512 - 32))
    return false;
  memcpy(sec + off, ent, 32);
  return img_write_sector(img, lba, sec);
}

static bool dir_iterate_root(fat16_image_t *img,
                             const uint8_t *match11,
                             bool want_free_slot,
                             dirent_ref_t *out_ref,
                             uint8_t out_ent[32])
{
  for (uint32_t i = 0; i < img->root_entries; i++)
  {
    uint32_t lba = img->root_lba + ((i * 32u) / 512u);
    uint16_t off = (uint16_t)((i * 32u) % 512u);
    uint8_t ent[32];
    if (!read_dirent_at(img, lba, off, ent))
      return false;

    uint8_t first = ent[0];
    if (want_free_slot)
    {
      if (first == 0x00 || first == 0xE5)
      {
        if (out_ref)
        {
          out_ref->lba = lba;
          out_ref->off = off;
        }
        if (out_ent)
          memcpy(out_ent, ent, 32);
        return true;
      }
      continue;
    }

    if (first == 0x00)
      return false;
    if (first == 0xE5)
      continue;
    if (ent[11] == 0x0F)
      continue;
    if (match11 && memcmp(ent, match11, 11) == 0)
    {
      if (out_ref)
      {
        out_ref->lba = lba;
        out_ref->off = off;
      }
      if (out_ent)
        memcpy(out_ent, ent, 32);
      return true;
    }
  }
  return false;
}

static bool dir_iterate_cluster_chain(fat16_image_t *img,
                                      uint16_t first_cluster,
                                      const uint8_t *match11,
                                      bool want_free_slot,
                                      dirent_ref_t *out_ref,
                                      uint8_t out_ent[32],
                                      bool *out_need_expand,
                                      uint16_t *out_last_cluster)
{
  uint16_t cluster = first_cluster;
  uint16_t last = first_cluster;
  while (cluster >= 2)
  {
    last = cluster;
    uint32_t base_lba = cluster_to_lba(img, cluster);
    for (uint8_t s = 0; s < img->sectors_per_cluster; s++)
    {
      uint32_t lba = base_lba + s;
      uint8_t sec[512];
      if (!img_read_sector(img, lba, sec))
        return false;
      for (uint16_t off = 0; off <= 512 - 32; off += 32)
      {
        const uint8_t *ent = sec + off;
        uint8_t first = ent[0];

        if (want_free_slot)
        {
          if (first == 0x00 || first == 0xE5)
          {
            if (out_ref)
            {
              out_ref->lba = lba;
              out_ref->off = off;
            }
            if (out_ent)
              memcpy(out_ent, ent, 32);
            if (out_need_expand)
              *out_need_expand = false;
            if (out_last_cluster)
              *out_last_cluster = last;
            return true;
          }
          continue;
        }

        if (first == 0x00)
        {
          if (out_need_expand)
            *out_need_expand = false;
          if (out_last_cluster)
            *out_last_cluster = last;
          return false;
        }
        if (first == 0xE5)
          continue;
        if (ent[11] == 0x0F)
          continue;
        if (match11 && memcmp(ent, match11, 11) == 0)
        {
          if (out_ref)
          {
            out_ref->lba = lba;
            out_ref->off = off;
          }
          if (out_ent)
            memcpy(out_ent, ent, 32);
          if (out_need_expand)
            *out_need_expand = false;
          if (out_last_cluster)
            *out_last_cluster = last;
          return true;
        }
      }
    }

    uint16_t next = 0;
    if (!fat_get(img, cluster, &next))
      return false;
    if (is_fat_eoc(next))
      break;
    cluster = next;
  }

  if (out_need_expand)
    *out_need_expand = true;
  if (out_last_cluster)
    *out_last_cluster = last;
  return false;
}

static bool resolve_dir(fat16_image_t *img, const char *path, dir_loc_t *out, char *err, size_t err_len)
{
  if (!img || !out)
  {
    set_err(err, err_len, "internal error");
    return false;
  }

  if (!path || path[0] == '\0' || (path[0] == '/' && path[1] == '\0'))
  {
    out->is_root = true;
    out->first_cluster = 0;
    return true;
  }

  char tmp[256];
  snprintf(tmp, sizeof(tmp), "%s", path);

  char *p = tmp;
  while (*p == '/')
    p++;

  dir_loc_t cur = {.is_root = true, .first_cluster = 0};
  while (*p)
  {
    char *slash = strchr(p, '/');
    if (slash)
      *slash = '\0';
    if (p[0] == '\0')
    {
      p = slash ? (slash + 1) : p + strlen(p);
      continue;
    }

    char name11[11];
    if (!name83_from_component(p, name11, err, err_len))
      return false;

    dirent_ref_t ref;
    uint8_t ent[32];
    bool found = cur.is_root ? dir_iterate_root(img, (uint8_t *)name11, false, &ref, ent)
                             : dir_iterate_cluster_chain(img, cur.first_cluster, (uint8_t *)name11, false, &ref, ent, NULL, NULL);
    if (!found)
    {
      set_err(err, err_len, "directory not found");
      return false;
    }
    if ((ent[11] & 0x10) == 0)
    {
      set_err(err, err_len, "not a directory");
      return false;
    }

    uint16_t cl = read_le16(ent + 26);
    cur.is_root = false;
    cur.first_cluster = cl;

    if (!slash)
      break;
    p = slash + 1;
    while (*p == '/')
      p++;
  }

  *out = cur;
  return true;
}

static bool split_parent(const char *path, char *out_parent, size_t parent_len, char *out_name, size_t name_len)
{
  if (!path || !out_parent || !out_name || parent_len == 0 || name_len == 0)
    return false;
  const char *last = strrchr(path, '/');
  if (!last)
  {
    snprintf(out_parent, parent_len, "/");
    snprintf(out_name, name_len, "%s", path);
    return true;
  }
  if (last == path)
  {
    snprintf(out_parent, parent_len, "/");
  }
  else
  {
    size_t n = (size_t)(last - path);
    if (n + 1 > parent_len)
      return false;
    memcpy(out_parent, path, n);
    out_parent[n] = '\0';
  }
  snprintf(out_name, name_len, "%s", last + 1);
  return true;
}

static bool alloc_cluster(fat16_image_t *img, uint16_t *out_cluster)
{
  if (!img || !out_cluster || !img->writeable)
    return false;

  const uint16_t start = (img->alloc_hint >= 2) ? img->alloc_hint : 2;
  uint16_t c = start;
  for (uint32_t scanned = 0; scanned < 65524u; scanned++)
  {
    uint16_t v = 0;
    if (!fat_get(img, c, &v))
      return false;
    if (v == 0x0000u)
    {
      if (!fat_set(img, c, 0xFFFFu))
        return false;
      img->alloc_hint = (uint16_t)(c + 1);
      *out_cluster = c;
      return true;
    }
    c++;
    if (c < 2)
      c = 2;
  }
  return false;
}

static void fat_free_chain(fat16_image_t *img, uint16_t first)
{
  if (!img || !img->writeable)
    return;
  uint16_t c = first;
  uint16_t guard = 0;
  while (c >= 2 && guard < 65524u)
  {
    uint16_t next = 0;
    if (!fat_get(img, c, &next))
      break;
    (void)fat_set(img, c, 0x0000u);
    if (is_fat_eoc(next))
      break;
    if (next == 0x0000u)
      break;
    c = next;
    guard++;
  }
}

static void fat_now(uint16_t *out_date, uint16_t *out_time)
{
  time_t now = time(NULL);
  struct tm *t = localtime(&now);
  int year = t ? (t->tm_year + 1900) : 1980;
  int mon = t ? (t->tm_mon + 1) : 1;
  int day = t ? t->tm_mday : 1;
  int hour = t ? t->tm_hour : 0;
  int min = t ? t->tm_min : 0;
  int sec = t ? t->tm_sec : 0;

  if (year < 1980)
    year = 1980;
  if (year > 2107)
    year = 2107;
  if (mon < 1)
    mon = 1;
  if (mon > 12)
    mon = 12;
  if (day < 1)
    day = 1;
  if (day > 31)
    day = 31;

  const uint16_t fat_date = (uint16_t)(((year - 1980) << 9) | (mon << 5) | day);
  const uint16_t fat_time = (uint16_t)((hour << 11) | (min << 5) | (sec / 2));
  if (out_date)
    *out_date = fat_date;
  if (out_time)
    *out_time = fat_time;
}

static bool read_sector0_partition(const uint8_t mbr[512], uint32_t *out_part_lba)
{
  if (!mbr || !out_part_lba)
    return false;
  if (mbr[510] != 0x55 || mbr[511] != 0xAA)
    return false;
  for (int i = 0; i < 4; i++)
  {
    const uint8_t *pte = mbr + 446 + i * 16;
    uint8_t type = pte[4];
    uint32_t lba = read_le32(pte + 8);
    uint32_t sectors = read_le32(pte + 12);
    if (type != 0 && lba != 0 && sectors != 0)
    {
      *out_part_lba = lba;
      return true;
    }
  }
  return false;
}

static bool parse_boot_sector(fat16_image_t *img, uint32_t part_lba, char *err, size_t err_len)
{
  uint8_t bs[512];
  if (!img_read_sector(img, part_lba, bs))
  {
    set_err(err, err_len, "boot sector read failed");
    return false;
  }
  if (bs[510] != 0x55 || bs[511] != 0xAA)
  {
    set_err(err, err_len, "invalid boot sector signature");
    return false;
  }

  uint16_t bps = read_le16(bs + 11);
  uint8_t spc = bs[13];
  uint16_t rs = read_le16(bs + 14);
  uint8_t fats = bs[16];
  uint16_t root = read_le16(bs + 17);
  uint16_t ts16 = read_le16(bs + 19);
  uint16_t spf = read_le16(bs + 22);
  uint32_t ts32 = read_le32(bs + 32);
  uint32_t total = ts16 ? ts16 : ts32;

  if (bps != 512 || spc == 0 || fats == 0 || root == 0 || spf == 0 || rs == 0 || total == 0)
  {
    set_err(err, err_len, "unsupported FAT layout");
    return false;
  }

  img->part_lba = part_lba;
  img->total_sectors = total;
  img->bytes_per_sector = bps;
  img->sectors_per_cluster = spc;
  img->reserved_sectors = rs;
  img->fats = fats;
  img->root_entries = root;
  img->sectors_per_fat = spf;

  img->fat_lba = img->part_lba + img->reserved_sectors;
  img->root_lba = img->fat_lba + (uint32_t)img->fats * (uint32_t)img->sectors_per_fat;
  img->root_sectors = ((uint32_t)img->root_entries * 32u + 511u) / 512u;
  img->data_lba = img->root_lba + img->root_sectors;
  img->alloc_hint = 2;
  return true;
}

bool fat16_image_open(fat16_image_t **out, const char *image_path, bool write, char *err, size_t err_len)
{
  if (!out || !image_path)
  {
    set_err(err, err_len, "invalid args");
    return false;
  }

  FILE *f = fopen(image_path, write ? "r+b" : "rb");
  if (!f)
  {
    set_err(err, err_len, "failed to open image");
    return false;
  }

  fat16_image_t *img = (fat16_image_t *)calloc(1, sizeof(*img));
  if (!img)
  {
    fclose(f);
    set_err(err, err_len, "oom");
    return false;
  }
  img->f = f;
  img->writeable = write;
  snprintf(img->path, sizeof(img->path), "%s", image_path);

  uint8_t sec0[512];
  if (!img_read_sector(img, 0, sec0))
  {
    fat16_image_close(img);
    set_err(err, err_len, "read failed");
    return false;
  }

  uint32_t part_lba = 0;
  if (!read_sector0_partition(sec0, &part_lba))
  {
    part_lba = 0;
  }

  if (!parse_boot_sector(img, part_lba, err, err_len))
  {
    fat16_image_close(img);
    return false;
  }

  *out = img;
  return true;
}

void fat16_image_close(fat16_image_t *img)
{
  if (!img)
    return;
  if (img->f)
  {
    fflush(img->f);
    fclose(img->f);
    img->f = NULL;
  }
  free(img);
}

static bool find_entry_in_dir(fat16_image_t *img,
                              const dir_loc_t *dir,
                              const uint8_t name11[11],
                              dirent_ref_t *out_ref,
                              uint8_t out_ent[32])
{
  if (!img || !dir || !name11)
    return false;
  if (dir->is_root)
  {
    return dir_iterate_root(img, name11, false, out_ref, out_ent);
  }
  return dir_iterate_cluster_chain(img, dir->first_cluster, name11, false, out_ref, out_ent, NULL, NULL);
}

static bool find_free_slot(fat16_image_t *img, const dir_loc_t *dir, dirent_ref_t *out_ref, char *err, size_t err_len)
{
  if (!img || !dir || !out_ref)
  {
    set_err(err, err_len, "internal error");
    return false;
  }
  if (dir->is_root)
  {
    uint8_t ent[32];
    if (!dir_iterate_root(img, NULL, true, out_ref, ent))
    {
      set_err(err, err_len, "root directory full");
      return false;
    }
    return true;
  }

  bool need_expand = false;
  uint16_t last_cluster = 0;
  uint8_t ent[32];
  if (dir_iterate_cluster_chain(img, dir->first_cluster, NULL, true, out_ref, ent, &need_expand, &last_cluster))
  {
    return true;
  }
  if (!need_expand)
  {
    set_err(err, err_len, "no free slot");
    return false;
  }

  uint16_t newc = 0;
  if (!alloc_cluster(img, &newc))
  {
    set_err(err, err_len, "no space");
    return false;
  }
  if (!fat_set(img, last_cluster, newc))
  {
    fat_free_chain(img, newc);
    set_err(err, err_len, "fat update failed");
    return false;
  }
  uint8_t zero[512];
  memset(zero, 0, sizeof(zero));
  uint32_t base = cluster_to_lba(img, newc);
  for (uint8_t s = 0; s < img->sectors_per_cluster; s++)
  {
    if (!img_write_sector(img, base + s, zero))
    {
      set_err(err, err_len, "write failed");
      return false;
    }
  }
  out_ref->lba = base;
  out_ref->off = 0;
  return true;
}

bool fat16_image_list_dir(fat16_image_t *img, const char *path, fat16_list_cb cb, void *arg, char *err, size_t err_len)
{
  if (!img || !cb)
  {
    set_err(err, err_len, "invalid args");
    return false;
  }

  dir_loc_t dir;
  if (!resolve_dir(img, path, &dir, err, err_len))
    return false;

  fat16_dirent_info_t info;
  memset(&info, 0, sizeof(info));

  if (dir.is_root)
  {
    for (uint32_t i = 0; i < img->root_entries; i++)
    {
      uint32_t lba = img->root_lba + ((i * 32u) / 512u);
      uint16_t off = (uint16_t)((i * 32u) % 512u);
      uint8_t ent[32];
      if (!read_dirent_at(img, lba, off, ent))
      {
        set_err(err, err_len, "read failed");
        return false;
      }
      if (ent[0] == 0x00)
        break;
      if (ent[0] == 0xE5)
        continue;
      if (ent[11] == 0x0F)
        continue;
      if (ent[11] & 0x08)
        continue;
      name83_to_string(ent, info.name);
      if (strcmp(info.name, ".") == 0 || strcmp(info.name, "..") == 0)
        continue;
      info.attr = ent[11];
      info.is_dir = (ent[11] & 0x10) != 0;
      info.fat_time = read_le16(ent + 22);
      info.fat_date = read_le16(ent + 24);
      info.size = read_le32(ent + 28);
      if (!cb(&info, arg))
        break;
    }
    return true;
  }

  uint16_t cluster = dir.first_cluster;
  uint16_t guard = 0;
  while (cluster >= 2 && guard < 65524u)
  {
    uint32_t base = cluster_to_lba(img, cluster);
    for (uint8_t s = 0; s < img->sectors_per_cluster; s++)
    {
      uint8_t sec[512];
      if (!img_read_sector(img, base + s, sec))
      {
        set_err(err, err_len, "read failed");
        return false;
      }
      for (uint16_t off = 0; off <= 512 - 32; off += 32)
      {
        uint8_t *ent = sec + off;
        if (ent[0] == 0x00)
          return true;
        if (ent[0] == 0xE5)
          continue;
        if (ent[11] == 0x0F)
          continue;
        if (ent[11] & 0x08)
          continue;
        name83_to_string(ent, info.name);
        if (strcmp(info.name, ".") == 0 || strcmp(info.name, "..") == 0)
          continue;
        info.attr = ent[11];
        info.is_dir = (ent[11] & 0x10) != 0;
        info.fat_time = read_le16(ent + 22);
        info.fat_date = read_le16(ent + 24);
        info.size = read_le32(ent + 28);
        if (!cb(&info, arg))
          return true;
      }
    }
    uint16_t next = 0;
    if (!fat_get(img, cluster, &next))
    {
      set_err(err, err_len, "fat read failed");
      return false;
    }
    if (is_fat_eoc(next))
      break;
    cluster = next;
    guard++;
  }
  return true;
}

bool fat16_image_mkdir(fat16_image_t *img, const char *dir_path, const char *name, char *err, size_t err_len)
{
  if (!img || !img->writeable)
  {
    set_err(err, err_len, "read-only");
    return false;
  }

  if (!dir_path || dir_path[0] == '\0')
  {
    dir_path = "/";
  }
  if (!name || name[0] == '\0')
  {
    set_err(err, err_len, "missing name");
    return false;
  }
  if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0 || strchr(name, '/'))
  {
    set_err(err, err_len, "invalid name");
    return false;
  }

  char name11[11];
  if (!name83_from_component(name, name11, err, err_len))
  {
    return false;
  }

  dir_loc_t dir;
  if (!resolve_dir(img, dir_path, &dir, err, err_len))
  {
    return false;
  }

  // Ensure it doesn't already exist.
  dirent_ref_t existing_ref;
  uint8_t existing_ent[32];
  bool exists = dir.is_root ? dir_iterate_root(img, (uint8_t *)name11, false, &existing_ref, existing_ent)
                            : dir_iterate_cluster_chain(img, dir.first_cluster, (uint8_t *)name11, false, &existing_ref,
                                                       existing_ent, NULL, NULL);
  if (exists)
  {
    set_err(err, err_len, "already exists");
    return false;
  }

  dirent_ref_t slot;
  if (!find_free_slot(img, &dir, &slot, err, err_len))
  {
    return false;
  }

  uint16_t new_cluster = 0;
  if (!alloc_cluster(img, &new_cluster))
  {
    set_err(err, err_len, "no space");
    return false;
  }

  uint8_t zero[512];
  memset(zero, 0, sizeof(zero));
  uint32_t base = cluster_to_lba(img, new_cluster);
  for (uint8_t s = 0; s < img->sectors_per_cluster; s++)
  {
    if (!img_write_sector(img, base + s, zero))
    {
      fat_free_chain(img, new_cluster);
      set_err(err, err_len, "write failed");
      return false;
    }
  }

  uint16_t fat_date = 0;
  uint16_t fat_time = 0;
  fat_now(&fat_date, &fat_time);

  uint8_t dot[32];
  memset(dot, 0, sizeof(dot));
  memset(dot, ' ', 11);
  dot[0] = '.';
  dot[11] = 0x10;
  write_le16(dot + 22, fat_time);
  write_le16(dot + 24, fat_date);
  write_le16(dot + 26, new_cluster);
  if (!write_dirent_at(img, base, 0, dot))
  {
    fat_free_chain(img, new_cluster);
    set_err(err, err_len, "write failed");
    return false;
  }

  uint8_t dotdot[32];
  memset(dotdot, 0, sizeof(dotdot));
  memset(dotdot, ' ', 11);
  dotdot[0] = '.';
  dotdot[1] = '.';
  dotdot[11] = 0x10;
  write_le16(dotdot + 22, fat_time);
  write_le16(dotdot + 24, fat_date);
  write_le16(dotdot + 26, dir.is_root ? 0 : dir.first_cluster);
  if (!write_dirent_at(img, base, 32, dotdot))
  {
    fat_free_chain(img, new_cluster);
    set_err(err, err_len, "write failed");
    return false;
  }

  uint8_t ent[32];
  memset(ent, 0, sizeof(ent));
  memcpy(ent, name11, 11);
  ent[11] = 0x10;
  write_le16(ent + 22, fat_time);
  write_le16(ent + 24, fat_date);
  write_le16(ent + 26, new_cluster);
  if (!write_dirent_at(img, slot.lba, slot.off, ent))
  {
    fat_free_chain(img, new_cluster);
    set_err(err, err_len, "write failed");
    return false;
  }

  return true;
}

bool fat16_image_delete(fat16_image_t *img, const char *path, char *err, size_t err_len)
{
  if (!img || !img->writeable || !path)
  {
    set_err(err, err_len, "invalid args");
    return false;
  }

  char parent[256];
  char name[64];
  if (!split_parent(path, parent, sizeof(parent), name, sizeof(name)))
  {
    set_err(err, err_len, "invalid path");
    return false;
  }

  dir_loc_t dir;
  if (!resolve_dir(img, parent, &dir, err, err_len))
    return false;

  char name11[11];
  if (!name83_from_component(name, name11, err, err_len))
    return false;

  dirent_ref_t ref;
  uint8_t ent[32];
  if (!find_entry_in_dir(img, &dir, (uint8_t *)name11, &ref, ent))
  {
    set_err(err, err_len, "not found");
    return false;
  }

  const bool is_dir = (ent[11] & 0x10) != 0;
  uint16_t first_cluster = read_le16(ent + 26);
  if (is_dir && first_cluster >= 2)
  {
    uint16_t cluster = first_cluster;
    uint16_t guard = 0;
    while (cluster >= 2 && guard < 65524u)
    {
      uint32_t base = cluster_to_lba(img, cluster);
      for (uint8_t s = 0; s < img->sectors_per_cluster; s++)
      {
        uint8_t sec[512];
        if (!img_read_sector(img, base + s, sec))
        {
          set_err(err, err_len, "read failed");
          return false;
        }
        for (uint16_t off = 0; off <= 512 - 32; off += 32)
        {
          uint8_t *e = sec + off;
          if (e[0] == 0x00)
            goto dir_empty_ok;
          if (e[0] == 0xE5 || e[11] == 0x0F)
            continue;
          char n[13];
          name83_to_string(e, n);
          if (strcmp(n, ".") == 0 || strcmp(n, "..") == 0)
            continue;
          set_err(err, err_len, "directory not empty");
          return false;
        }
      }
      uint16_t next = 0;
      if (!fat_get(img, cluster, &next))
      {
        set_err(err, err_len, "fat read failed");
        return false;
      }
      if (is_fat_eoc(next))
        break;
      cluster = next;
      guard++;
    }
  }

dir_empty_ok:
  if (first_cluster >= 2)
    fat_free_chain(img, first_cluster);

  ent[0] = 0xE5;
  if (!write_dirent_at(img, ref.lba, ref.off, ent))
  {
    set_err(err, err_len, "write failed");
    return false;
  }
  fflush(img->f);
  return true;
}

bool fat16_image_rename(fat16_image_t *img, const char *path, const char *new_name, char *err, size_t err_len)
{
  if (!img || !img->writeable || !path || !new_name)
  {
    set_err(err, err_len, "invalid args");
    return false;
  }

  char parent[256];
  char name[64];
  if (!split_parent(path, parent, sizeof(parent), name, sizeof(name)))
  {
    set_err(err, err_len, "invalid path");
    return false;
  }

  dir_loc_t dir;
  if (!resolve_dir(img, parent, &dir, err, err_len))
    return false;

  char old11[11];
  if (!name83_from_component(name, old11, err, err_len))
    return false;

  char new11[11];
  if (!name83_from_component(new_name, new11, err, err_len))
    return false;

  if (memcmp(old11, new11, 11) != 0)
  {
    if (find_entry_in_dir(img, &dir, (uint8_t *)new11, NULL, NULL))
    {
      set_err(err, err_len, "target exists");
      return false;
    }
  }

  dirent_ref_t ref;
  uint8_t ent[32];
  if (!find_entry_in_dir(img, &dir, (uint8_t *)old11, &ref, ent))
  {
    set_err(err, err_len, "not found");
    return false;
  }

  memcpy(ent, new11, 11);
  if (!write_dirent_at(img, ref.lba, ref.off, ent))
  {
    set_err(err, err_len, "write failed");
    return false;
  }
  fflush(img->f);
  return true;
}

static bool ensure_file_slot_and_clear_existing(fat16_image_t *img,
                                               const dir_loc_t *dir,
                                               const char name11[11],
                                               dirent_ref_t *out_slot,
                                               char *err,
                                               size_t err_len)
{
  dirent_ref_t existing_ref;
  uint8_t existing_ent[32];
  if (find_entry_in_dir(img, dir, (const uint8_t *)name11, &existing_ref, existing_ent))
  {
    if (existing_ent[11] & 0x10)
    {
      set_err(err, err_len, "cannot overwrite directory");
      return false;
    }
    uint16_t first_cluster = read_le16(existing_ent + 26);
    if (first_cluster >= 2)
      fat_free_chain(img, first_cluster);
    existing_ent[0] = 0xE5;
    if (!write_dirent_at(img, existing_ref.lba, existing_ref.off, existing_ent))
    {
      set_err(err, err_len, "write failed");
      return false;
    }
    *out_slot = existing_ref;
    return true;
  }

  if (!find_free_slot(img, dir, out_slot, err, err_len))
    return false;
  return true;
}

bool fat16_image_write_begin(fat16_image_t *img,
                            const char *dir_path,
                            const char *name,
                            uint32_t total_size,
                            fat16_write_ctx_t *out,
                            char *err,
                            size_t err_len)
{
  if (!img || !img->writeable || !out || !name)
  {
    set_err(err, err_len, "invalid args");
    return false;
  }

  dir_loc_t dir;
  if (!resolve_dir(img, dir_path, &dir, err, err_len))
    return false;

  char name11[11];
  if (!name83_from_component(name, name11, err, err_len))
    return false;

  dirent_ref_t slot;
  if (!ensure_file_slot_and_clear_existing(img, &dir, name11, &slot, err, err_len))
    return false;

  memset(out, 0, sizeof(*out));
  out->img = img;
  out->dirent_lba = slot.lba;
  out->dirent_off = slot.off;
  out->size = total_size;
  out->active = true;

  uint8_t ent[32];
  memset(ent, 0, sizeof(ent));
  memcpy(ent, name11, 11);
  ent[11] = 0x20; // archive
  uint16_t fd = 0, ft = 0;
  fat_now(&fd, &ft);
  write_le16(ent + 22, ft);
  write_le16(ent + 24, fd);
  write_le16(ent + 26, 0);
  write_le32(ent + 28, 0);
  if (!write_dirent_at(img, slot.lba, slot.off, ent))
  {
    set_err(err, err_len, "write failed");
    return false;
  }

  return true;
}

static bool ensure_active_cluster(fat16_write_ctx_t *ctx, char *err, size_t err_len)
{
  if (!ctx || !ctx->img)
  {
    set_err(err, err_len, "internal error");
    return false;
  }
  if (ctx->first_cluster >= 2)
    return true;

  uint16_t c = 0;
  if (!alloc_cluster(ctx->img, &c))
  {
    set_err(err, err_len, "no space");
    return false;
  }
  ctx->first_cluster = c;
  ctx->last_cluster = c;
  ctx->cluster_bytes_used = 0;
  ctx->sector_used = 0;
  ctx->sector_in_cluster = 0;
  return true;
}

static bool advance_cluster_if_needed(fat16_write_ctx_t *ctx, char *err, size_t err_len)
{
  const uint32_t cluster_size = (uint32_t)ctx->img->sectors_per_cluster * 512u;
  if (ctx->cluster_bytes_used < cluster_size)
    return true;

  if (ctx->sector_used != 0)
  {
    set_err(err, err_len, "internal write state");
    return false;
  }

  uint16_t next = 0;
  if (!alloc_cluster(ctx->img, &next))
  {
    set_err(err, err_len, "no space");
    return false;
  }
  if (!fat_set(ctx->img, ctx->last_cluster, next))
  {
    fat_free_chain(ctx->img, next);
    set_err(err, err_len, "fat update failed");
    return false;
  }
  ctx->last_cluster = next;
  ctx->cluster_bytes_used = 0;
  ctx->sector_in_cluster = 0;
  return true;
}

static bool flush_sector(fat16_write_ctx_t *ctx, const uint8_t *sector512, char *err, size_t err_len)
{
  uint32_t base = cluster_to_lba(ctx->img, ctx->last_cluster);
  uint32_t lba = base + ctx->sector_in_cluster;
  if (!img_write_sector(ctx->img, lba, sector512))
  {
    set_err(err, err_len, "write failed");
    return false;
  }
  ctx->sector_in_cluster++;
  ctx->cluster_bytes_used += 512u;
  return true;
}

bool fat16_image_write_append(fat16_write_ctx_t *ctx, const uint8_t *data, size_t len, char *err, size_t err_len)
{
  if (!ctx || !ctx->active)
  {
    set_err(err, err_len, "not active");
    return false;
  }
  if (!data && len != 0)
  {
    set_err(err, err_len, "invalid args");
    return false;
  }

  size_t off = 0;
  while (off < len)
  {
    if (!ensure_active_cluster(ctx, err, err_len))
      return false;
    if (!advance_cluster_if_needed(ctx, err, err_len))
      return false;

    const size_t space = (size_t)(512u - ctx->sector_used);
    const size_t take = (len - off < space) ? (len - off) : space;
    memcpy(ctx->sector_buf + ctx->sector_used, data + off, take);
    ctx->sector_used = (uint16_t)(ctx->sector_used + take);
    off += take;

    if (ctx->sector_used == 512u)
    {
      if (!flush_sector(ctx, ctx->sector_buf, err, err_len))
        return false;
      ctx->sector_used = 0;
    }
  }

  return true;
}

bool fat16_image_write_finish(fat16_write_ctx_t *ctx, char *err, size_t err_len)
{
  if (!ctx || !ctx->active || !ctx->img)
  {
    set_err(err, err_len, "not active");
    return false;
  }

  if (ctx->sector_used != 0)
  {
    memset(ctx->sector_buf + ctx->sector_used, 0, 512u - ctx->sector_used);
    if (!ensure_active_cluster(ctx, err, err_len))
      return false;
    if (!advance_cluster_if_needed(ctx, err, err_len))
      return false;
    if (!flush_sector(ctx, ctx->sector_buf, err, err_len))
      return false;
    ctx->sector_used = 0;
  }

  if (ctx->first_cluster >= 2)
  {
    uint8_t zero[512];
    memset(zero, 0, sizeof(zero));
    while (ctx->sector_in_cluster < ctx->img->sectors_per_cluster)
    {
      if (!flush_sector(ctx, zero, err, err_len))
        return false;
    }
  }

  uint8_t ent[32];
  if (!read_dirent_at(ctx->img, ctx->dirent_lba, ctx->dirent_off, ent))
  {
    set_err(err, err_len, "read failed");
    return false;
  }
  uint16_t fd = 0, ft = 0;
  fat_now(&fd, &ft);
  write_le16(ent + 22, ft);
  write_le16(ent + 24, fd);
  write_le16(ent + 26, ctx->first_cluster);
  write_le32(ent + 28, ctx->size);
  if (!write_dirent_at(ctx->img, ctx->dirent_lba, ctx->dirent_off, ent))
  {
    set_err(err, err_len, "write failed");
    return false;
  }

  fflush(ctx->img->f);
  ctx->active = false;
  return true;
}

void fat16_image_write_abort(fat16_write_ctx_t *ctx)
{
  if (!ctx || !ctx->active || !ctx->img)
    return;

  if (ctx->first_cluster >= 2)
  {
    fat_free_chain(ctx->img, ctx->first_cluster);
  }

  uint8_t ent[32];
  if (read_dirent_at(ctx->img, ctx->dirent_lba, ctx->dirent_off, ent))
  {
    ent[0] = 0xE5;
    (void)write_dirent_at(ctx->img, ctx->dirent_lba, ctx->dirent_off, ent);
  }
  fflush(ctx->img->f);
  ctx->active = false;
}
