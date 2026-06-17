#set text(font: "New Computer Modern", size: 10pt, lang: "id")
#set par(justify: true, first-line-indent: (all: true, amount: 1.8em))
#set heading(numbering: "1.1.")
#set math.equation(numbering: "(1)")
#set page(
  paper: "a4",
  margin: (left: 20mm, right: 20mm, top: 35mm, bottom: 20mm),
  header-ascent: 12mm,
  header: context {
    stack(
      dir: ttb,
      grid(
        columns: (1fr, 1fr),
        align(left)[#image("ITS_pojok.pdf", width: 2cm)], // Pastikan file logo ada di folder yang sama
        align(right)[#text(size: 10pt)[#datetime.today().display("[year]-[month]-[day]")]]
      ),
      v(0.4em),
      line(length: 100%, stroke: 0.5pt)
    )
  }
)

// Bagian Judul
#align(center)[
  #v(1em)
  #text(size: 18pt, weight: "bold")[Laporan Eksplorasi dan Pemetaan Ruang B-202 dengan Pioneer P3DX berbasis Semantic SLAM]
  #v(1em)
]

// Tabel Metadata
#grid(
  columns: (auto, 1fr),
  gutter: 1em,
  [*Penulis*], [: Rayhan Rizqi Zamzamy],
  [*Dosen*], [: Muhammad Qomaruz Zaman, S.T., M.T., Ph.D.],
  [*Nama Kelas*], [: Sistem Robot Otonom],
  [*Link video YouTube*], [: #link("https://youtu.be/_undvDP7nWw")],
  [], [#hide()[: ] #link("https://youtu.be/hFe5IIcUWlc")],
  [*Link GitHub*], [: #link("https://github.com/hanzamzamy/SRO26_Assignment/tree/fp")]
)

#v(1.5cm)

= Pendahuluan
Laporan ini menyajikan desain komprehensif dari sistem navigasi otonom pada _mobile robot differential drive_ Pioneer P3DX di dalam lingkungan CoppeliaSim. Tujuan utama dari tugas ini adalah melakukan pemetaan lingkungan (_mapping_) menggunakan susunan sensor ultrasonik, merancang jalur eksplorasi otonom hibrida, dan mengimplementasikan pelacakan jalur tingkat lanjut (_advanced path tracking_).

Lebih lanjut, sistem ini dilengkapi dengan kapabilitas _Semantic SLAM_ menggunakan integrasi _Vision-Language Model_ (VLM). Robot tidak hanya menghindari rintangan, tetapi juga mampu membangun basis data memori spasial dari _vision sensor_ dan merespons perintah navigasi berbasis bahasa alami (_Natural Language_) melalui arsitektur _sense-plan-act_ secara _real-time_.

= Pemodelan dan Algoritma Navigasi
Pemodelan dan analisis kinematika robot didasarkan pada literatur standar robotika @lynch2017modern @siciliano2008springer. Sistem navigasi dibagi menjadi empat subsistem utama: Pemetaan _grid_ okupansi, Perencanaan jalur A\*, Pengendali _pure pursuit_ termodifikasi, dan Agen Logika Semantik.

== Pemetaan Lingkungan (_Occupancy Grid Mapping_)
Pemetaan dilakukan menggunakan 9 buah sensor ultrasonik bagian depan P3DX. Ruang simulasi didiskritisasi menjadi _grid_ 2D beresolusi $0.1 "m/sel"$. Pembaruan probabilitas keberadaan rintangan menggunakan pendekatan _log-odds_ untuk menghindari _underflow_ numerik.

Transformasi koordinat pantulan sensor ($p_S$) dari kerangka lokal sensor ke kerangka dunia absolut ($p_W$) didefinisikan sebagai,
$ p_W = ""^W T_B dot ""^B T_S dot p_S $ <eq:transform>
#h(-1.8em)Dimana $""^W T_B$ adalah matriks transformasi dari koordinat dunia ke bodi robot, dan $""^B T_S$ adalah transformasi dari bodi ke sensor. Pembaruan probabilitas menggunakan algoritma garis Bresenham dan model _inverse sensor_ Bayesian klasik.

== Perencanaan Jalur Jarak Terpendek (A\* dengan _Gradient Costmap_)
Peta diekspansi (_inflation_) membentuk _gradient costmap_ secara sirkular. Radius fisik robot ($r_"lethal" = 0.3 "m"$) diberi nilai penalti $100.0$, merepresentasikan tidak dapat dilewati. Area penyangga keamanan ($r_"risk" = 0.5 "m"$) diberikan penalti yang menurun secara linear. Algoritma A\* meminimalkan fungsi biaya total $f(n) = g(n) + h(n) + (C_"risk" times 2.0)$, memaksa robot merencanakan jalur di tengah ruangan terbuka.

== Pelacakan Jalur (_Pure Pursuit_ dengan _Point-Turn_)
Pengontrol _pure pursuit_ dimodifikasi dengan penambahan batas toleransi putar-di-tempat (_point-turn_) untuk mengatasi rintangan tajam. Kecepatan sudut rotasi robot ($omega$) dikalkulasi secara proporsional terhadap kelengkungan $gamma$. Jika target berada pada sudut tajam > 0.6 radian, robot akan melakukan _point-turn_.

== Arsitektur Semantic SLAM dan Vision-Language Model
Sistem ini menggunakan VLM untuk memproses informasi visual dan tekstual, terbagi dalam dua fase utama.

1. *Fase Eksplorasi Berjenjang (_Macro_ & _Frontier_):* Robot memadukan dua strategi. Pertama, eksplorasi makro dengan menargetkan titik jauh di luar peta untuk memaksa sapuan jarak jauh. Ketika ruangan telah tertutup (_sealed map_), robot beralih ke _Frontier Exploration_, secara matematis mencari sel matriks yang berbatasan antara area bebas dan area belum terpetakan. Selama fase ini, gambar dari kamera diproses oleh VLM untuk diekstrak menjadi _landmark_ dan disimpan dalam JSON memori spasial berserta koordinat dan orientasinya (_yaw_).
2. *Fase Eksekusi (NLP & Verifikasi Visual):* Perintah bahasa alami dari pengguna (misal: "Hampiri meja bundar") diproses oleh VLM untuk mengekstraksi niat (_intent_) dan objek target murni. Robot mencari target di memori, merutekan koordinat aman menggunakan fungsi _Safe Parking Area_ agar tidak menabrak rintangan, dan terakhir memverifikasi kembali secara visual (`verify_target_presence`) setelah tiba di lokasi.

= Implementasi dan Pengujian Sistem
Sistem ini diuji dari tahap fundamental pemetaan manual hingga mode kognitif otonom yang digerakkan oleh AI.

#figure(
  image("final_map_bak.png", width: 50%),
  caption: [Proses pemetaan manual tahap awal. Garis hitam merupakan dinding (probabilitas tinggi), area putih adalah ruang bebas.],
) <fig:map_manual>

== Pengujian Eksplorasi Otonom dan Resolusi Tabrakan
Implementasi antarmuka _matplotlib_ bersifat interaktif, menunjukkan status _state machine_ secara langsung. Sistem dilengkapi dengan _software bumper_ yang memantau perubahan posisi. Jika robot mendeteksi posisi bergerak kurang dari $0.03 "m"$ dalam $1.5$ detik saat motor menyala, robot akan memundurkan roda ($v = -1.0$) dan memaksa A\* melakukan _replanning_ dinamis.

#figure(
  grid(
    columns: (1fr, 1fr),
    align(right)[#text(size: 9pt)[
      #image("qabla_jalan_singkat.png", width: 50%)
      ]],
    align(left)[#text(size: 9pt)[
      #image("bada_jalan_singkat.png", width: 50%)
      ]]
  ),
  caption: [Kiri: Jalur A\* awal yang direncanakan. Kanan: Jalur A\* yang diperbarui secara dinamis saat menemui rintangan tak terpetakan (_live replanning_).],
) <fig:shortest_path>

== Pengujian _Semantic SLAM_ (VLM)
Pengujian fase logik semantik membuktikan robot mampu memadukan _machine vision_ dengan _natural language processing_ (NLP).
#figure(
  image("logging_memory.png", width: 60%),
  caption: [Proses _logging_ memori. Robot secara otomatis memindai gambar dan menyimpannya ke basis data spasial.],
) <fig:memory_log>

Sistem memecahkan masalah navigasi klasik (di mana menugaskan robot ke koordinat objek akan membuatnya menabrak benda tersebut) dengan fungsi kalkulasi _Nearest Safe Cell_. Ini memastikan bahwa A\* mengarahkan robot untuk parkir dengan aman di sebelah rintangan (titik terdekat berpenalti rendah).

#figure(
  image("nlp_exec.png", width: 60%),
  caption: [Robot menerima perintah dalam bahasa alami, mengekstrak objek, melakukan perjalanan ke sel teraman, dan memverifikasi keberadaan target dengan kamera sebelum berhenti.],
) <fig:verification>

= Kesimpulan
Sistem navigasi otonom yang dikembangkan telah berevolusi dari sekadar pemetaan rintangan menjadi _Semantic SLAM_ kognitif tertutup. Kombinasi dari eksplorasi _macro_ dan _frontier_ menjamin efisiensi penjelajahan lingkungan tertutup, sementara inflasi _gradient costmap_ pada A\* berhasil menavigasi robot melintasi masalah lorong sempit (_narrow passages_). Puncak dari sistem ini adalah integrasi _Vision-Language Model_ yang mengubah robot menjadi asisten spasial, yang tidak hanya menyadari hambatan fisik di sekitarnya, tetapi juga memahami makna dan nama dari objek-objek tersebut secara alami.

#bibliography("references.bib", style: "ieee")
